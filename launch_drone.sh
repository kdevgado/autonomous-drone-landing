#!/bin/bash

set -e

colcon build
source install/setup.bash

# Define each package and its launch command
declare -A PACKAGE_COMMANDS
PACKAGE_COMMANDS["rplidar_ros"]="ros2 launch rplidar_ros rplidar_a3_launch.py"
PACKAGE_COMMANDS["ardupilot_cartographer"]="ros2 launch ardupilot_cartographer cartographer.launch.py"
PACKAGE_COMMANDS["landing_pad_detector"]="ros2 run landing_pad_detector detector"
# PACKAGE_COMMANDS["jetson_monitor"]="ros2 launch data_monitor jetson_monitor_node"
# PACKAGE_COMMANDS["drone_monitor"]="ros2 launch data_monitor drone_monitor_node"
PACKAGE_COMMANDS["foxglove_bridge"]="ros2 launch foxglove_bridge foxglove_bridge_launch.xml"
PACKAGE_COMMANDS["mavros"]="ros2 run mavros mavros_node --ros-args -p fcu_url:=serial:///dev/ttyACM0:57600"

echo "Do you want to start the entire drone system? (y/n)"
read -r FULL_START

if [[ "$FULL_START" == "y" || "$FULL_START" == "Y" ]]; then
    echo "Starting entire drone system..."
    for pkg in "${!PACKAGE_COMMANDS[@]}"; do
        echo "Launching $pkg..."
        ${PACKAGE_COMMANDS[$pkg]} &
    done
else
    echo "Select packages to start:"
    for pkg in "${!PACKAGE_COMMANDS[@]}"; do
        echo "Start package '$pkg'? (y/n)"
        read -r answer
        if [[ "$answer" == "y" || "$answer" == "Y" ]]; then
            echo "Launching $pkg..."
            ${PACKAGE_COMMANDS[$pkg]} &
        fi
    done
fi

wait
