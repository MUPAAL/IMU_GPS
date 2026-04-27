#!/usr/bin/env bash
# start_bridges.sh — Start 01~04, 06 bridges in separate tmux windows

set -euo pipefail

SESSION="bridges"
ROOT="$(cd "$(dirname "$0")" && pwd)"

start_window() {
    local window_name="$1"
    local command="$2"

    if [[ -z "${FIRST_WINDOW_STARTED:-}" ]]; then
        tmux new-session -d -s "$SESSION" -n "$window_name" "$command"
        FIRST_WINDOW_STARTED=1
    else
        tmux new-window -t "$SESSION" -n "$window_name" "$command"
    fi
}

# Attach to existing session if it exists
if tmux has-session -t "$SESSION" 2>/dev/null; then
    echo "Session '$SESSION' already exists — attaching..."
    tmux attach-session -t "$SESSION"
    exit 0
fi

start_window "01_IMU" "cd '$ROOT/01_IMU' && python imu_bridge.py"
start_window "02_RTK" "cd '$ROOT/02_RTK' && python rtk_bridge.py"
start_window "03_Nav" "cd '$ROOT/03_Nav' && python nav_bridge.py"
start_window "04_Robot" "cd '$ROOT/04_Robot' && python robot_bridge.py"
start_window "05_AutoNav" "cd '$ROOT/05_AutoNav' && python autonav_bridge.py"
start_window "06_Camera" "cd '$ROOT/06_Camera' && python camera_bridge.py"

tmux select-window -t "$SESSION:01_IMU"

# Attach to session
tmux attach-session -t "$SESSION"
