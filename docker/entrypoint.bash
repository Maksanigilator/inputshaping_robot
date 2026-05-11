#!/usr/bin/env bash
# Container entrypoint: source ROS, build workspace (idempotent), then exec.
#
# Intentionally does NOT use `set -e` / `set -u`: ROS 2 humble's setup.bash
# and a number of ament shell snippets reference unset variables and ignore
# minor non-zero returns. With strict modes the whole sourcing chain bails
# out *silently* (subshell exits without printing why), which is what
# previously left AMENT_PREFIX_PATH pointing only at /opt/ros/humble.
set -o pipefail

# Predeclare optional ament trace var so any helper sourcing it under -u
# downstream doesn't blow up.
export AMENT_TRACE_SETUP_FILES="${AMENT_TRACE_SETUP_FILES:-}"

log() { printf '[entrypoint] %s\n' "$*"; }
die() { printf '[entrypoint] ERROR: %s\n' "$*" >&2; exit 1; }

# --- 1. Source the base ROS install. ----------------------------------------
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
if [[ -z "${AMENT_PREFIX_PATH:-}" ]]; then
    die "/opt/ros/humble/setup.bash did not set AMENT_PREFIX_PATH"
fi

cd /workspace

# --- 2. Build the workspace if needed (or if --rebuild was passed). --------
if [[ "${1:-}" == "--rebuild" ]]; then
    shift
    log "rebuild requested; removing build/install/log"
    rm -rf build install log
fi

# File whose presence proves the install/ contains a fully-built copy of
# our package (and not a leftover from a build that aborted mid-way).
PKG_MARKER="install/inputshaping_core/share/ament_index/resource_index/packages/inputshaping_core"

if [[ ! -f install/setup.bash || ! -f "$PKG_MARKER" ]]; then
    log "install/ is missing or incomplete -> wiping and rebuilding"
    # NB: build/install/log are bind-mounted (named-volume mountpoints in our
    # compose). We can't `rm -rf` the dirs themselves -- only their contents.
    find build install log -mindepth 1 -delete 2>/dev/null || true
    if ! colcon build --symlink-install; then
        die "colcon build failed"
    fi
fi

if [[ ! -f install/setup.bash || ! -f "$PKG_MARKER" ]]; then
    die "install still incomplete after build; check colcon output above"
fi

# --- 3. Overlay the workspace. ---------------------------------------------
# shellcheck disable=SC1091
source install/setup.bash

# --- 4. Make data dir writable for the non-root host user. -----------------
mkdir -p /workspace/data
chmod 0777 /workspace/data || true

# --- 5. Diagnostics. Print where each package was found from so a missing
# overlay shows up immediately in the logs.
log "AMENT_PREFIX_PATH=$AMENT_PREFIX_PATH"
if ros2 pkg prefix inputshaping_core >/dev/null 2>&1; then
    log "inputshaping_core resolved at: $(ros2 pkg prefix inputshaping_core)"
else
    log "WARNING: inputshaping_core NOT visible to ros2; ros2 pkg list output follows:"
    ros2 pkg list 2>&1 | sed 's/^/    /' | head -20 || true
    log "(continuing -- the launch step will fail next, but with more context)"
fi

# --- 6. Exec the user command. ---------------------------------------------
exec "$@"
