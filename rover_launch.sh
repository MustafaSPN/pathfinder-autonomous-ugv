#!/bin/bash

# --- AYARLAR ---
SESSION="rover_mission"
CONTAINER="ros2_humble"
DELAY=5

# HER PENCERE ICIN GECERLI ROS AYARLARI (BURAYA EKLENDI)
# Bu komut zinciri her pane acildiginda calisacak
ROS_ENV_VARS="export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && export ROS_DOMAIN_ID=0 && export ROS_LOCALHOST_ONLY=0"
# ---------------

# Eski session varsa temizle
tmux kill-session -t $SESSION 2>/dev/null

echo "Rover Sistemleri Baslatiliyor..."

# ---------------------------------------------------------
# 1. PENCERE (SOL UST): Micro-ROS
# ---------------------------------------------------------
tmux new-session -d -s $SESSION
tmux rename-window -t $SESSION:0 'Rover Mission'

tmux send-keys "docker exec -it $CONTAINER bash" C-m
sleep $DELAY
# Once kaynak dosyasi, sonra bizim ozel ayarlarimiz
tmux send-keys "cd ~/microros_ws/ && source install/setup.bash && $ROS_ENV_VARS" C-m
tmux send-keys "ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 --baudrate 115200" C-m


# ---------------------------------------------------------
# 2. PENCERE (SAG UST): Rover Bringup
# ---------------------------------------------------------
tmux split-window -h 

tmux send-keys "docker exec -it $CONTAINER bash" C-m
sleep $DELAY
# Once kaynak dosyasi, sonra bizim ozel ayarlarimiz
tmux send-keys "cd ~/ros2_ws/ && source install/setup.bash && $ROS_ENV_VARS" C-m
tmux send-keys "ros2 launch rover_bringup rover.launch.py" C-m


# ---------------------------------------------------------
# 3. PENCERE (SOL ALT): Navigation
# ---------------------------------------------------------
tmux select-pane -t 0
tmux split-window -v

tmux send-keys "docker exec -it $CONTAINER bash" C-m
sleep $DELAY
# Once kaynak dosyasi, sonra bizim ozel ayarlarimiz
tmux send-keys "cd ~/ros2_ws/ && source install/setup.bash && $ROS_ENV_VARS" C-m
tmux send-keys "ros2 launch rover_bringup navigation.launch.py" C-m


# ---------------------------------------------------------
# 4. PENCERE (SAG ALT): Web Backend
# ---------------------------------------------------------
tmux select-pane -t 0
tmux select-pane -R
tmux split-window -v

tmux send-keys "docker exec -it $CONTAINER bash" C-m
sleep $DELAY
# Once kaynak dosyasi, sonra bizim ozel ayarlarimiz
tmux send-keys "cd ~/ros2_ws/ && source install/setup.bash && $ROS_ENV_VARS" C-m
tmux send-keys "cd apps/pathfinder_webapp/backend/" C-m
tmux send-keys "python3 -m uvicorn app:app --host 0.0.0.0 --port 8000" C-m


# ---------------------------------------------------------
# BITIRIS
# ---------------------------------------------------------
tmux select-layout tiled
tmux set -g mouse on

tmux select-pane -t 0
tmux attach-session -t $SESSION