#!/usr/bin/env bash
# Convenience wrapper for talking to the robot host (aida@192.168.31.91)
# from this laptop. The Pico + the local `inputshaping` container live here,
# the rest of the stack (cmd_vel_mux, odroid_driver, ds4_driver) runs on
# the robot. Both sides share ROS_DOMAIN_ID=0 with rmw_fastrtps_cpp and
# are on the same wifi subnet, so DDS multicast does the integration --
# we never have to ship our package to the robot itself.
#
# Subcommands cheat-sheet:
#   status            -- containers, mux mode, our nodes/topics
#   topics            -- ros2 topic list on the robot
#   mux               -- current cmd_vel_mux/mode + watchdog rate
#   nodes             -- ros2 node list on the robot
#   echo <topic>      -- ros2 topic echo run on the robot side
#   hz   <topic>      -- ros2 topic hz on the robot side
#   info <topic>      -- ros2 topic info --verbose on the robot
#   logs <ctr>        -- docker logs --tail=200 -f for <ctr>
#   compose-up        -- (re)start aida_bot_ws on the robot
#   compose-down      -- stop aida_bot_ws on the robot
#   exec <ctr> ...    -- docker exec into a container on the robot
#   ssh               -- open an interactive shell on the robot

set -u

ROBOT_USER="${ROBOT_USER:-aida}"
ROBOT_HOST="${ROBOT_HOST:-192.168.31.91}"
# The ROS-aware container that already has ros2 CLI sourced.
ROS_CTR="${ROS_CTR:-aida_bot_ws-odroid_node-1}"

ssh_to() { ssh -o ConnectTimeout=5 "${ROBOT_USER}@${ROBOT_HOST}" "$@"; }
in_ros() {
    # Run a snippet inside the ROS container with /opt/ros/humble sourced.
    # We send the script over stdin via heredoc so we don't have to worry
    # about how many layers of zsh/bash/docker mangle our quotes.
    local snippet="$1"
    ssh -o ConnectTimeout=5 "${ROBOT_USER}@${ROBOT_HOST}" \
        "docker exec -i ${ROS_CTR} bash -l" <<__EOF
source /opt/ros/humble/setup.bash >/dev/null 2>&1
${snippet}
__EOF
}

cmd="${1:-status}"
shift || true

case "$cmd" in
status)
    echo "=== robot containers ==="
    ssh_to "docker ps --format 'table {{.Names}}\t{{.Status}}\t{{.Image}}'"
    echo
    echo "=== cmd_vel_mux/mode (one-shot) ==="
    in_ros "timeout 2 ros2 topic echo --once /cmd_vel_mux/mode 2>&1 | head -3"
    echo
    echo "=== nodes containing 'inputshaping' or 'mpu' (visible via DDS) ==="
    in_ros "ros2 node list 2>&1 | grep -E 'inputshaping|mpu' || echo NONE_VISIBLE_is_the_laptop_container_up"
    ;;
topics)        in_ros "ros2 topic list 2>&1 | sort" ;;
nodes)         in_ros "ros2 node list 2>&1 | sort" ;;
mux)
    in_ros "echo 'mode topic:'; timeout 2 ros2 topic echo --once /cmd_vel_mux/mode 2>&1 | head -3; \
            echo; echo 'mux params:'; ros2 param dump /cmd_vel_mux 2>&1 | head -40"
    ;;
echo)
    [ -n "${1:-}" ] || { echo "usage: robot.sh echo <topic>"; exit 2; }
    in_ros "ros2 topic echo $1"
    ;;
hz)
    [ -n "${1:-}" ] || { echo "usage: robot.sh hz <topic>"; exit 2; }
    in_ros "ros2 topic hz $1"
    ;;
info)
    [ -n "${1:-}" ] || { echo "usage: robot.sh info <topic>"; exit 2; }
    in_ros "ros2 topic info --verbose $1"
    ;;
logs)
    ctr="${1:-${ROS_CTR}}"
    ssh_to "docker logs --tail=200 -f $ctr"
    ;;
compose-up)
    # The robot keeps aida_bot_ws in ~/projects/aida_bot_ws.
    ssh_to "cd ~/projects/aida_bot_ws && docker compose up -d"
    ;;
compose-down)
    ssh_to "cd ~/projects/aida_bot_ws && docker compose down"
    ;;
exec)
    [ -n "${1:-}" ] || { echo "usage: robot.sh exec <container> <cmd...>"; exit 2; }
    ctr="$1"; shift
    if [ $# -eq 0 ]; then
        # Interactive shell -- request a TTY both at ssh and at docker exec.
        ssh -t "${ROBOT_USER}@${ROBOT_HOST}" "docker exec -it $ctr bash"
    else
        # One-shot command: pipe argv as stdin so we don't need a TTY and
        # don't have to wrestle with quoting across three shells.
        printf '%s\n' "$*" | ssh "${ROBOT_USER}@${ROBOT_HOST}" \
            "docker exec -i $ctr bash -l"
    fi
    ;;
ssh)
    ssh "${ROBOT_USER}@${ROBOT_HOST}"
    ;;
*)
    sed -n '1,40p' "$0"
    exit 2
    ;;
esac
