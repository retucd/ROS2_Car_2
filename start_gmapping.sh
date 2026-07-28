#!/bin/bash

export NO_AT_BRIDGE=1

source /opt/ros/humble/setup.bash
source ~/Robot_Car/install/setup.bash

gnome-terminal --title="base_driver" -- bash -c "
source /opt/ros/humble/setup.bash
source ~/Robot_Car/install/setup.bash

ros2 launch car_base_driver base_driver.launch.py

exec bash
"

sleep 3

gnome-terminal --title="ydlidar" -- bash -c "
source /opt/ros/humble/setup.bash
source ~/Robot_Car/install/setup.bash

ros2 launch ydlidar_ros2_driver ydlidar_launch.py

exec bash
"

sleep 3

gnome-terminal --title="robot_model" -- bash -c "
source /opt/ros/humble/setup.bash
source ~/Robot_Car/install/setup.bash

ros2 launch car_model_urdf display_xacro.launch.py

exec bash
"

sleep 2

gnome-terminal --title="ekf" -- bash -c "
source /opt/ros/humble/setup.bash
source ~/Robot_Car/install/setup.bash

ros2 run robot_localization ekf_node \
  --ros-args \
  --params-file /home/sunrise/Robot_Car/install/car_navigation/share/car_navigation/config/ekf.yaml \
  -r /odometry/filtered:=/odom

exec bash
"

sleep 3

gnome-terminal --title="gmapping" -- bash -c "
source /opt/ros/humble/setup.bash
source ~/Robot_Car/install/setup.bash

ros2 launch slam_gmapping slam_gmapping.launch.py

exec bash
"