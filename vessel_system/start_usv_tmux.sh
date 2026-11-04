#!/usr/bin/env bash

set -euo pipefail

# ============================================================================
# USV / UAV SIMULATION STARTUP
# Creates one tmux session with:
#
#   1. Control terminal
#   2. Gazebo / ROS simulation
#   3. X500 waypoint manager
#   4. Motor mixer
#   5. Controller
#   6. USV waypoint manager
#
# Startup sequence:
#
#   t = 0 s   -> Simulation
#   t = 10 s  -> X500 waypoint manager
#   t = 12 s  -> Motor mixer + Controller
#   t = 42 s  -> USV waypoint manager
# ============================================================================


# Tmux session name
SESSION="usv_sim"


# ============================================================================
# CHECK TMUX
# ============================================================================

# Stop immediately if tmux is not installed.
if ! command -v tmux >/dev/null 2>&1; then
    echo "ERROR: tmux is not installed."
    echo "Install it with:"
    echo "  apt update && apt install -y tmux"
    exit 1
fi


# ============================================================================
# ROS 2 ENVIRONMENT
# ============================================================================

# ROS 2 workspaces and the current graphical environment must be available in
# every pane. The packages may be installed together in ros2_ws (the Docker
# layout) or split between ros2_ws and usv_ws (the two existing machines).
# Quoted values also avoid inheriting stale variables from a long-lived tmux
# server.
printf -v DISPLAY_VALUE '%q' "${DISPLAY:-:0}"
SOURCE_CMD="export DISPLAY=$DISPLAY_VALUE"

if [[ -n "${XDG_RUNTIME_DIR:-}" ]]; then
    printf -v XDG_RUNTIME_DIR_VALUE '%q' "$XDG_RUNTIME_DIR"
    SOURCE_CMD+=" && export XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR_VALUE"
fi

if [[ -n "${WAYLAND_DISPLAY:-}" ]]; then
    printf -v WAYLAND_DISPLAY_VALUE '%q' "$WAYLAND_DISPLAY"
    SOURCE_CMD+=" && export WAYLAND_DISPLAY=$WAYLAND_DISPLAY_VALUE"
fi

SOURCE_CMD+=" && source /opt/ros/humble/setup.bash"

WORKSPACE_FOUND=false
for SETUP_FILE in \
    /root/ros2_ws/install/setup.bash \
    /root/usv_ws/install/setup.bash
do
    if [[ -f "$SETUP_FILE" ]]; then
        printf -v SETUP_FILE_VALUE '%q' "$SETUP_FILE"
        SOURCE_CMD+=" && source $SETUP_FILE_VALUE"
        WORKSPACE_FOUND=true
    fi
done

if [[ "$WORKSPACE_FOUND" != true ]]; then
    echo "ERROR: no ROS 2 workspace setup file was found."
    echo "Build ros2_ws or usv_ws before starting the simulation."
    exit 1
fi


# ============================================================================
# CLEAN OLD SESSION
# ============================================================================

# If a tmux session with the same name already exists, close only that session.
if tmux has-session -t "$SESSION" 2>/dev/null; then
    echo "Closing existing tmux session: $SESSION"
    tmux kill-session -t "$SESSION"
    sleep 1
fi


# ============================================================================
# CREATE TMUX SESSION
# ============================================================================

# Create the tmux server and the first pane.
# This first pane will be used as the empty control terminal.
# A detached tmux session otherwise starts at 80x24. That is too small for six
# panes and makes the last split fail with "no space for new pane".
tmux new-session -d -x 240 -y 60 -s "$SESSION" -n "simulation"

# Make sure the session was actually created.
if ! tmux has-session -t "$SESSION" 2>/dev/null; then
    echo "ERROR: Failed to create tmux session."
    exit 1
fi

# Get the ID of the first pane.
CONTROL_PANE=$(tmux display-message -p -t "$SESSION:simulation.0" '#{pane_id}')


# ============================================================================
# CREATE THE OTHER FIVE TERMINALS
# ============================================================================

# Create terminal for Gazebo / simulation.
SIM_PANE=$(tmux split-window \
    -d \
    -t "$CONTROL_PANE" \
    -P \
    -F '#{pane_id}')

# Create terminal for X500 waypoint manager.
X500_WP_PANE=$(tmux split-window \
    -d \
    -t "$CONTROL_PANE" \
    -P \
    -F '#{pane_id}')

# Create terminal for motor mixer.
MOTOR_MIXER_PANE=$(tmux split-window \
    -d \
    -t "$CONTROL_PANE" \
    -P \
    -F '#{pane_id}')

# Create terminal for controller.
CONTROLLER_PANE=$(tmux split-window \
    -d \
    -t "$CONTROL_PANE" \
    -P \
    -F '#{pane_id}')

# Create terminal for USV waypoint manager.
USV_WP_PANE=$(tmux split-window \
    -d \
    -t "$CONTROL_PANE" \
    -P \
    -F '#{pane_id}')


# Arrange all six terminals automatically.
tmux select-layout -t "$SESSION:simulation" tiled


# ============================================================================
# TERMINAL 1 - CONTROL
# ============================================================================

# Empty control terminal.
# Both ROS 2 workspaces are already sourced and it is ready for manual commands.
tmux send-keys -t "$CONTROL_PANE" \
    "$SOURCE_CMD && clear && echo '[CONTROL] ROS 2 workspaces sourced. Terminal ready.'" \
    C-m


# ============================================================================
# TERMINAL 2 - SIMULATION
# t = 0 seconds
# ============================================================================

# Start Gazebo and the ROS 2 simulation immediately.
tmux send-keys -t "$SIM_PANE" \
    "$SOURCE_CMD && echo '[SIMULATION] Starting simulation...' && exec ros2 launch asv_simulator sim.launch.py" \
    C-m


# ============================================================================
# TERMINAL 3 - X500 WAYPOINT MANAGER
# t = 10 seconds
# ============================================================================

# Wait for Gazebo and ROS interfaces to initialize before starting UAV control.
tmux send-keys -t "$X500_WP_PANE" \
    "$SOURCE_CMD && echo '[X500 WP] Waiting 10 seconds...' && sleep 10 && echo '[X500 WP] Starting...' && exec ros2 run vessel_system x500wp_manager" \
    C-m


# ============================================================================
# TERMINAL 4 - MOTOR MIXER
# t = 12 seconds
# ============================================================================

# Start two seconds after the X500 waypoint manager.
tmux send-keys -t "$MOTOR_MIXER_PANE" \
    "$SOURCE_CMD && echo '[MOTOR MIXER] Waiting 12 seconds...' && sleep 12 && echo '[MOTOR MIXER] Starting...' && exec ros2 run vessel_system motor_mixer" \
    C-m


# ============================================================================
# TERMINAL 5 - CONTROLLER
# t = 12 seconds
# ============================================================================

# Start simultaneously with the motor mixer.
tmux send-keys -t "$CONTROLLER_PANE" \
    "$SOURCE_CMD && echo '[CONTROLLER] Waiting 12 seconds...' && sleep 12 && echo '[CONTROLLER] Starting...' && exec ros2 run vessel_system controller_node" \
    C-m


# ============================================================================
# TERMINAL 6 - USV WAYPOINT MANAGER
# t = 22 seconds
# ============================================================================

# Start ten seconds after motor mixer and controller.
tmux send-keys -t "$USV_WP_PANE" \
    "$SOURCE_CMD && echo '[USV WP] Waiting 42 seconds...' && sleep 42 && echo '[USV WP] Starting...' && exec ros2 run vessel_system waypoint_manager_node" \
    C-m


# ============================================================================
# ATTACH TO SESSION
# ============================================================================

# Select the empty control terminal by default.
tmux select-pane -t "$CONTROL_PANE"

# Attach to the newly created tmux session.
tmux attach-session -t "$SESSION"
