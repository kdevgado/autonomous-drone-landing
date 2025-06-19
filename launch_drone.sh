#!/bin/bash

set -e
source /opt/ros/humble/setup.bash
source /opt/venv/bin/activate
cd /ros2_ws
colcon build
source install/setup.bash

# Define each package and its launch command
declare -A PACKAGE_COMMANDS
PACKAGE_COMMANDS["rplidar_ros"]="ros2 launch rplidar_ros rplidar_a3_launch.py"
PACKAGE_COMMANDS["ardupilot_cartographer"]="ros2 launch ardupilot_cartographer cartographer.launch.py"
PACKAGE_COMMANDS["landing_pad_detector"]="ros2 run landing_pad_detector detector_node"
# PACKAGE_COMMANDS["jetson_monitor"]="ros2 launch data_monitor jetson_monitor_node"
# PACKAGE_COMMANDS["drone_monitor"]="ros2 launch data_monitor drone_monitor_node"
PACKAGE_COMMANDS["foxglove_bridge"]="ros2 launch foxglove_bridge foxglove_bridge_launch.xml"
PACKAGE_COMMANDS["mavros"]="ros2 run mavros mavros_node --ros-args -p fcu_url:=serial:///dev/ttyACM0:57600"

echo "Do you want to start the entire drone system? (y/n)"
read -r FULL_START

# Ask for debug mode
echo "Enable debug mode (show output in new terminal)? (y/n)"
read -r DEBUG_MODE

# If debug mode, create tmux session first
if [[ "$DEBUG_MODE" == "y" || "$DEBUG_MODE" == "Y" ]]; then
	tmux list-sessions
    tmux has-session -t drone_launcher 2>/dev/null && tmux kill-session -t drone_launcher
	tmux list-sessions
    tmux new-session -d -s drone_launcher -n starter "echo 'Starting drone packages...'; sleep 3"
	tmux list-sessions
fi

start_package() {
    local pkg=$1
    echo "Launching $pkg..."

    local cmd="${PACKAGE_COMMANDS[$pkg]}"

    if [[ "$DEBUG_MODE" == "y" || "$DEBUG_MODE" == "Y" ]]; then
        # Create a new tmux window in the master session for this package  
	tmux list-sessions
	tmux split-window -t drone_launcher -h "$cmd"
	tmux select-layout -t drone_launcher tiled

 
    else
        bash -c "$cmd" > /dev/null 2>&1 &
    fi

    sleep 1

    if pgrep -f "$cmd" > /dev/null; then
        echo "✔️  $pkg started (debug: $DEBUG_MODE)"
    else
        echo "❌  Failed to start $pkg"
    fi
}

if [[ "$FULL_START" == "y" || "$FULL_START" == "Y" ]]; then
    echo "Starting entire drone system..."
    for pkg in "${!PACKAGE_COMMANDS[@]}"; do
	    start_package "$pkg"
    done
else
    echo "Select packages to start:"
    for pkg in "${!PACKAGE_COMMANDS[@]}"; do
        echo "Start package '$pkg'? (y/n)"
        read -r answer
        if [[ "$answer" == "y" || "$answer" == "Y" ]]; then
		start_package "$pkg"
        fi
    done
fi

if [[ "$DEBUG_MODE" == "y" || "$DEBUG_MODE" == "Y" ]]; then
    #tmux kill-window -t drone_launcher:starter 2>/dev/null
    tmux attach-session -t drone_launcher
fi

