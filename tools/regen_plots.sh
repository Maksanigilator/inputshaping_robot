#!/usr/bin/env bash
# Re-render PSD/shaper PNGs from existing calibration CSVs using the
# project's Docker image. Bypasses the ROS entrypoint (we only need
# numpy/scipy/matplotlib + our own inputshaping_core sources -- no
# colcon build, no install/ volume).
#
# Usage:
#   tools/regen_plots.sh data/calibration_data_x_20260511_170551.csv [more.csv ...]
#
# Adds --user $UID:$GID so the resulting PNGs land on disk owned by you
# rather than root.

set -euo pipefail

repo_root="$(cd "$(dirname "$0")/.." && pwd)"

if [[ "$#" -lt 1 ]]; then
    echo "usage: $0 <calibration_data_csv> [more.csv ...]" >&2
    echo "       paths are interpreted relative to the repo root, or absolute" >&2
    exit 2
fi

mapped=()
for arg in "$@"; do
    case "$arg" in
        /*)
            # Absolute path: must live under repo_root because we only
            # bind-mount that subtree.
            rel="${arg#"$repo_root/"}"
            [[ "$rel" == "$arg" ]] && { echo "path is outside repo root: $arg" >&2; exit 2; }
            mapped+=("/workspace/$rel")
            ;;
        *)
            mapped+=("/workspace/$arg")
            ;;
    esac
done

docker run --rm \
    -v "$repo_root/src:/workspace/src:ro" \
    -v "$repo_root/data:/workspace/data" \
    -v "$repo_root/tools:/workspace/tools:ro" \
    -e PYTHONPATH=/workspace/src/inputshaping_core \
    -e MPLBACKEND=Agg \
    -e MPLCONFIGDIR=/tmp/mpl \
    --user "$(id -u):$(id -g)" \
    --entrypoint python3 \
    inputshaping_robot:humble \
    /workspace/tools/regen_plots.py "${mapped[@]}"
