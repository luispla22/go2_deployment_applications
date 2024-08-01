#!/bin/bash
source /home/unitree/unitree_ros2/cyclonedds_ws/src/install/setup.bash
ros2 bag record -a -o 'rosbag_latest.bag'