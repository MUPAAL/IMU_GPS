#!/usr/bin/env bash
# start_helpers.sh — launch temporary helper tools (listen/replay/send) in tmux
#
# Usage:
#   ./start_helpers.sh
#   # then input selection like: 234 or 2 3 4
#
# Notes:
# - Runs each selected tool in its module directory.
# - Creates/attaches tmux session: helpers

set -euo pipefail

SESSION="helpers"
ROOT="$(cd "$(dirname "$0")" && pwd)"

show_menu() {
  cat <<'EOF'

Temporary Helpers Menu
======================
1  01_IMU   listen
2  01_IMU   replay
3  02_RTK   listen
4  02_RTK   replay
5  03_Nav   listen
6  03_Nav   replay
7  04_Robot listen
8  04_Robot send_demo
9  04_Robot replay
A  06_Camera listen
B  06_Camera replay
C  03_Nav   bridge
D  03_Nav   full replay stack (2+4+C+5)
E  09_PathFollow bridge
F  09_PathFollow listen

X  Stop helpers session
Q  Quit

Input examples:
  234      (start items 2,3,4)
  2 3 4    (same)
  7A       (start robot listen + camera listen)
EOF
}

cmd_for_key() {
  local key="$1"
  case "$key" in
    1) echo "cd '$ROOT/01_IMU' && python listen_imu_websocket.py" ;;
    2) echo "cd '$ROOT/01_IMU' && python replay_imu_websocket.py" ;;
    3) echo "cd '$ROOT/02_RTK' && python listen_rtk_websocket.py" ;;
    4) echo "cd '$ROOT/02_RTK' && python replay_rtk_websocket.py" ;;
    5) echo "cd '$ROOT/03_Nav' && python listen_nav_websocket.py" ;;
    6) echo "cd '$ROOT/03_Nav' && python replay_nav_websocket.py" ;;
    7) echo "cd '$ROOT/04_Robot' && python listen_robot_websocket.py" ;;
    8) echo "cd '$ROOT/04_Robot' && python send_robot_only_demo.py" ;;
    9) echo "cd '$ROOT/04_Robot' && python replay_robot_websocket.py" ;;
    A) echo "cd '$ROOT/06_Camera' && python listen_camera_websocket.py" ;;
    B) echo "cd '$ROOT/06_Camera' && python replay_camera_websocket.py" ;;
    C) echo "cd '$ROOT/03_Nav' && python nav_bridge.py" ;;
    E) echo "cd '$ROOT/09_PathFollow' && python path_follow_bridge.py" ;;
    F) echo "cd '$ROOT/09_PathFollow' && python listen_path_follow_websocket.py" ;;
    *) echo "" ;;
  esac
}

start_selected() {
  local selection="$1"
  local normalized
  normalized="$(echo "$selection" | tr '[:lower:]' '[:upper:]' | tr -d '[:space:]')"

  if [[ -z "$normalized" ]]; then
    echo "No selection."
    return 0
  fi

  # Preset: full Nav replay stack
  if [[ "$normalized" == "D" ]]; then
    normalized="24C5"
    echo "Preset D -> $normalized"
  fi

  # If Nav bridge is selected, auto-add IMU/RTK replay dependencies.
  if [[ "$normalized" == *"C"* ]]; then
    if [[ "$normalized" != *"2"* ]]; then
      normalized="2$normalized"
    fi
    if [[ "$normalized" != *"4"* ]]; then
      normalized="4$normalized"
    fi
    echo "Auto-added dependencies for C: selection -> $normalized"
  fi

  if tmux has-session -t "$SESSION" 2>/dev/null; then
    tmux kill-session -t "$SESSION"
  fi

  local first=1
  local key cmd
  local -a started=()

  # iterate one character at a time
  for (( i=0; i<${#normalized}; i++ )); do
    key="${normalized:i:1}"
    cmd="$(cmd_for_key "$key")"
    if [[ -z "$cmd" ]]; then
      echo "Skip unknown key: $key"
      continue
    fi

    if [[ $first -eq 1 ]]; then
      tmux new-session -d -s "$SESSION" -n "helpers" "$cmd"
      first=0
    else
      tmux split-window -t "$SESSION:0" "$cmd"
    fi
    started+=("$key")
  done

  if [[ ${#started[@]} -eq 0 ]]; then
    echo "No valid task selected."
    return 0
  fi

  tmux select-layout -t "$SESSION:0" tiled >/dev/null
  echo "Started helpers: ${started[*]}"
  tmux attach-session -t "$SESSION"
}

show_menu
read -r -p "Select: " selection
selection_upper="$(echo "$selection" | tr '[:lower:]' '[:upper:]' | tr -d '[:space:]')"

case "$selection_upper" in
  X)
    if tmux has-session -t "$SESSION" 2>/dev/null; then
      tmux kill-session -t "$SESSION"
      echo "Session '$SESSION' stopped."
    else
      echo "Session '$SESSION' not running."
    fi
    ;;
  Q)
    echo "Bye."
    ;;
  *)
    start_selected "$selection"
    ;;
esac
