#!/bin/bash

export NO_AT_BRIDGE=1

source /opt/ros/humble/setup.bash
source ~/Robot_Car/install/setup.bash

gnome-terminal --title="cartographer_mapping" -- bash -c "
source /opt/ros/humble/setup.bash
source ~/Robot_Car/install/setup.bash

ros2 launch car_navigation cartographer_mapping.launch.py \
  use_sim_time:=false \
  base_port:=/dev/ttyS3

exec bash
"