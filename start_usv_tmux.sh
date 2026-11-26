#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

# Default: run the Nautilus mission through the three red markers in ocean_world.
# Flags can be combined.
#   ./start_usv_tmux.sh --headless --detach
#   ./start_usv_tmux.sh --rviz            # Gazebo plus RViz
#   ./start_usv_tmux.sh --rviz-only       # RViz with the Gazebo server in background
#   ./start_usv_tmux.sh --vessel-v2 --x500  # both USVs plus the UAV
#   ./start_usv_tmux.sh --no-nautilus --vessel-v2
#
# Model flags enable both the Gazebo spawn and the matching control chain:
#   Nautilus / vessel_v2 -> surface_control.launch.py and one waypoint manager
#                           per model, isolated by ROS namespace.
#   X500               -> x500wp_manager
#   BlueROV2 Heavy     -> uuv_controller in waypoint mode
#   DURIUS             -> spawn only (no controller configured here).
# To test manual surface waypoints, stop that boat's waypoint-manager pane
# and publish geometry_msgs/msg/Point to /<model>/target_waypoint.
# Both managers accept waypoints_xy to override the mission. These are x/y
# pairs in metres in ocean_world, regardless of the model's spawn position.
# Pass --external-control=<model> to reserve its actuators for an external
# controller. The model must also be enabled using its spawn flag.

SESSION="${USV_SIM_SESSION:-usv_sim}"
NAUTILUS=true
VESSEL_V2=false
X500=false
UUV=false
DURIUS=false
HEADLESS=false
START_GZ_GUI=true
START_RVIZ=false
RVIZ_CONFIG=vessel_viz.rviz
DETACH=false
declare -A EXTERNAL_CONTROL=()

for option in "$@"; do
    case "$option" in
        --nautilus) NAUTILUS=true ;;
        --no-nautilus) NAUTILUS=false ;;
        --vessel-v2) VESSEL_V2=true ;;
        --x500) X500=true ;;
        --uuv) UUV=true ;;
        --durius) DURIUS=true ;;
        --external-control=vessel_v2|--external-control=nautilus_vessel|--external-control=x500|--external-control=uuv_bluerov2_heavy)
            EXTERNAL_CONTROL["${option#*=}"]=true ;;
        --headless) HEADLESS=true ;;
        --rviz) START_GZ_GUI=true; START_RVIZ=true; RVIZ_CONFIG=vessel_viz.rviz ;;
        --rviz-only) START_GZ_GUI=false; START_RVIZ=true; RVIZ_CONFIG=surface_tracking.rviz ;;
        --no-rviz) START_RVIZ=false ;;
        --detach) DETACH=true ;;
        -h|--help)
            echo "Usage: $0 [--no-nautilus|--nautilus] [--vessel-v2] [--x500] [--uuv] [--durius] [--headless] [--rviz|--rviz-only|--no-rviz] [--detach]"
            echo "Nautilus and its automatic mission through the three red markers are enabled by default."
            echo "--external-control=MODEL disables local control for an enabled model."
            exit 0 ;;
        *) echo "Unknown option: $option" >&2; exit 2 ;;
    esac
done

local_control() {
    if [[ "$2" == true && "${EXTERNAL_CONTROL[$1]:-false}" == false ]]; then
        printf true
    else
        printf false
    fi
}

CONTROL_NAUTILUS=$(local_control nautilus_vessel "$NAUTILUS")
CONTROL_VESSEL_V2=$(local_control vessel_v2 "$VESSEL_V2")
CONTROL_X500=$(local_control x500 "$X500")
CONTROL_UUV=$(local_control uuv_bluerov2_heavy "$UUV")

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

printf -v SCRIPT_DIR_VALUE '%q' "$SCRIPT_DIR"
SOURCE_CMD+=" && export PATH=$SCRIPT_DIR_VALUE:\$PATH"

# Inherited by child processes even if they detach from their launch pane.
read -r SIM_RUN_ID < /proc/sys/kernel/random/uuid
SOURCE_CMD+=" && export ASV_SIM_RUN_ID=$SIM_RUN_ID"

# Isolate discovery from previous runs and other Gazebo instances. Set
# GZ_PARTITION explicitly only when an external Gazebo client must share it.
SIM_PARTITION="${GZ_PARTITION:-asv_simulator_$SIM_RUN_ID}"
printf -v GZ_PARTITION_VALUE '%q' "$SIM_PARTITION"
SOURCE_CMD+=" && export GZ_PARTITION=$GZ_PARTITION_VALUE"

if [[ -n "${XAUTHORITY:-}" ]]; then
    printf -v XAUTHORITY_VALUE '%q' "$XAUTHORITY"
    SOURCE_CMD+=" && export XAUTHORITY=$XAUTHORITY_VALUE"
fi

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
    TKILL_DETACHED=1 "$SCRIPT_DIR/tkill" "$SESSION"
fi


# ============================================================================
# CREATE TMUX SESSION
# ============================================================================

# Create the tmux server and the first pane.
# This first pane will be used as the empty control terminal.
# Start with enough room for the optional model panes.
SESSION_ID="$(tmux new-session -d -P -F '#{session_id}' \
    -x 240 -y 60 -s "$SESSION" -n "simulation")"
tmux set-option -t "$SESSION_ID" @asv_sim_run_id "$SIM_RUN_ID"

# Make sure the session was actually created.
if ! tmux has-session -t "$SESSION" 2>/dev/null; then
    echo "ERROR: Failed to create tmux session."
    exit 1
fi

# Get the ID of the first pane.
CONTROL_PANE=$(tmux display-message -p -t "$SESSION:simulation.0" '#{pane_id}')

# Keep a pane for each enabled control chain. Retile after each split so
# enabling several optional models does not exhaust the first pane's height.
start_pane() {
    local pane
    pane=$(tmux split-window -d -t "$CONTROL_PANE" -P -F '#{pane_id}')
    tmux select-layout -t "$SESSION:simulation" tiled
    tmux send-keys -t "$pane" "$SOURCE_CMD && exec $1" C-m
}

tmux send-keys -t "$CONTROL_PANE" "$SOURCE_CMD" C-m

# The managers wait for GPS/odometry and republish the current waypoint.
# The controllers wait for feedback, so fixed startup sleeps are unnecessary.
start_pane "ros2 launch asv_simulator sim.launch.py headless:=$HEADLESS start_gz_gui:=$START_GZ_GUI start_rviz:=$START_RVIZ rviz_config:=$RVIZ_CONFIG spawn_nautilus:=$NAUTILUS spawn_vessel_v2:=$VESSEL_V2 spawn_uav:=$X500 x500_external_control:=${EXTERNAL_CONTROL[x500]:-false} spawn_uuv:=$UUV spawn_durius:=$DURIUS"

if [[ "$CONTROL_NAUTILUS" == true || "$CONTROL_VESSEL_V2" == true ]]; then
    start_pane "ros2 launch vessel_system surface_control.launch.py control_nautilus_vessel:=$CONTROL_NAUTILUS control_vessel_v2:=$CONTROL_VESSEL_V2"
fi
if [[ "$CONTROL_NAUTILUS" == true ]]; then
    start_pane "ros2 run vessel_system waypoint_manager_node --ros-args -r __ns:=/nautilus_vessel -p model:=nautilus_vessel"
fi
if [[ "$CONTROL_VESSEL_V2" == true ]]; then
    start_pane "ros2 run vessel_system waypoint_manager_node --ros-args -r __ns:=/vessel_v2 -p model:=vessel_v2"
fi
if [[ "$CONTROL_X500" == true ]]; then
    start_pane "ros2 run vessel_system x500wp_manager"
fi
if [[ "$CONTROL_UUV" == true ]]; then
    start_pane "ros2 run vessel_system uuv_controller --ros-args -p start_mode:=waypoint"
fi

tmux select-pane -t "$CONTROL_PANE"
if [[ "$DETACH" == true ]]; then
    echo "Simulation started. Attach with: tmux attach-session -t $SESSION"
    echo "Gazebo partition: $SIM_PARTITION"
else
    tmux attach-session -t "$SESSION"
fi
