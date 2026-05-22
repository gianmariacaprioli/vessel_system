#!/bin/bash

# Nome della sessione tmux
SESSION="vrx_session"
WS_DIR="/root/vessel_ws"

# Comando per fare il source dell'ambiente in ogni pannello
# (Se usi una distro diversa da humble, cambiala qui)
SETUP_CMD="source /opt/ros/humble/setup.bash && source ${WS_DIR}/install/setup.bash"

# 1. Crea una nuova sessione tmux in background
tmux new-session -d -s $SESSION -c $WS_DIR

# --- COSTRUZIONE DEL LAYOUT ---
# Dividi lo schermo orizzontalmente: Sopra (Pannello 0) e Sotto (Pannello 1)
tmux split-window -v -p 40

# Seleziona la riga superiore (0) e dividila in 3 colonne
tmux select-pane -t 0
tmux split-window -h -p 66
tmux split-window -h -p 50

# Seleziona la riga inferiore (ora è diventata il pannello 3) e dividila a metà
tmux select-pane -t 3
tmux split-window -h -p 50

# --- ESECUZIONE DEI COMANDI ---
# Pannello 0 (In alto a sinistra) - TF / Odometria
tmux send-keys -t 0 "$SETUP_CMD" C-m
tmux send-keys -t 0 "ros2 run vessel_system pose_to_tf --ros-args -p use_sim_time:=true" C-m

# Pannello 1 (In alto al centro) - SLAM Toolbox
tmux send-keys -t 1 "$SETUP_CMD" C-m
tmux send-keys -t 1 "ros2 run slam_toolbox async_slam_toolbox_node --ros-args -p use_sim_time:=true -p odom_frame:=odom -p base_frame:=wamv/wamv/base_link -p map_frame:=map -p scan_topic:=/wamv/sensors/lidars/lidar_wamv_sensor/scan -p mode:=mapping" C-m

# Pannello 2 (In alto a destra) - VRX Simulatore
tmux send-keys -t 2 "$SETUP_CMD" C-m
tmux send-keys -t 2 "ros2 launch vrx_gz competition.launch.py" C-m

# Pannello 3 (In basso a sinistra) - RViz2 con configurazione custom
tmux send-keys -t 3 "$SETUP_CMD" C-m
tmux send-keys -t 3 "rviz2 -d /root/vessel_ws/src/vessel_system/config/rviz2_vessel.rviz" C-m

# Pannello 4 (In basso a destra) - Test Driver
tmux send-keys -t 4 "$SETUP_CMD" C-m
tmux send-keys -t 4 "ros2 run vessel_system test_driver" C-m

# 2. Agganciati alla sessione appena creata
tmux attach-session -t $SESSION