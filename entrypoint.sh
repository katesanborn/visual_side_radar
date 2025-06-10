#!/bin/bash
source /opt/ros/noetic/setup.bash
source /catkin_ws/devel/setup.bash
exec roslaunch radar_display radar_display.launch "$@"
