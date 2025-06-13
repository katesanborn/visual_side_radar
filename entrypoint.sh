#!/bin/bash
source /opt/ros/noetic/setup.bash
source /catkin_ws/devel/setup.bash
exec roslaunch visual_side_radar radar_display.launch "$@"
