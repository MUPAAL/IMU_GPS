#!/usr/bin/env bash
# start_bridges.sh — Start 01~04, 06 bridges in a tmux session with tiled layout
#
# Layout (2x2 + 1):
#   ┌──────────────┬──────────────┐
#   │  01_IMU      │  02_RTK      │
#   ├──────────────┼──────────────┤
#   │  03_Nav      │  04_Robot    │
#   ├──────────────┼──────────────┤
#   │  06_Camera              │
#   └──────────────┴──────────────┘

SESSION="bridges"
ROOT="$(cd "$(dirname "$0")" && pwd)"

# Attach to existing session if it exists
if tmux has-session -t "$SESSION" 2>/dev/null; then
    echo "Session '$SESSION' already exists — attaching..."
    tmux attach-session -t "$SESSION"
    exit 0
fi

# Create session with first pane = 01_IMU
tmux new-session -d -s "$SESSION" -n "bridges" \
    -x "$(tput cols)" -y "$(tput lines)"

# Pane 0.0 — 01_IMU (top-left, exists by default)
tmux send-keys -t "$SESSION:0.0" \
    "cd '$ROOT/01_IMU' && python imu_bridge.py" Enter

# Pane 0.1 — 02_RTK (top-right, split horizontally)
tmux split-window -t "$SESSION:0.0" -h \
    "cd '$ROOT/02_RTK' && python rtk_bridge.py"

# Pane 0.2 — 03_Nav (middle-left, split vertically from 0.0)
tmux split-window -t "$SESSION:0.0" -v \
    "cd '$ROOT/03_Nav' && python nav_bridge.py"

# Pane 0.3 — 04_Robot (middle-right, split vertically from 0.1)
tmux split-window -t "$SESSION:0.1" -v \
    "cd '$ROOT/04_Robot' && python robot_bridge.py"

# Pane 0.4 — 06_Camera (bottom, split vertically from 0.2)
tmux split-window -t "$SESSION:0.2" -v \
    "cd '$ROOT/06_Camera' && python camera_bridge.py"

# Arrange all panes uniformly in a grid
tmux select-layout -t "$SESSION:0" tiled

# Set pane titles (requires terminal support for title escape)
tmux select-pane -t "$SESSION:0.0" -T "01_IMU"
tmux select-pane -t "$SESSION:0.1" -T "02_RTK"
tmux select-pane -t "$SESSION:0.2" -T "03_Nav"
tmux select-pane -t "$SESSION:0.3" -T "04_Robot"
tmux select-pane -t "$SESSION:0.4" -T "06_Camera"

# Focus top-left pane
tmux select-pane -t "$SESSION:0.0"

# Attach to session
tmux attach-session -t "$SESSION"
