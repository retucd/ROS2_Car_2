#!/bin/bash

export NO_AT_BRIDGE=1

source /opt/ros/humble/setup.bash
source "$HOME/Robot_Car/install/setup.bash"

BASE_PORT=/dev/ttyS3

gnome-terminal --title="ydlidar" -- bash -c "
source /opt/ros/humble/setup.bash
source \"\$HOME/Robot_Car/install/setup.bash\"

ros2 launch ydlidar_ros2_driver ydlidar_launch.py

exec bash
"

sleep 3

gnome-terminal --title="navigation" -- bash -c "
source /opt/ros/humble/setup.bash
source \"\$HOME/Robot_Car/install/setup.bash\"

ros2 launch car_navigation navigation.launch.py \
  base_port:=$BASE_PORT

exec bash
"

echo "Navigation started. Start RViz in the virtual machine:"
echo "ros2 launch car_navigation rviz.launch.py"