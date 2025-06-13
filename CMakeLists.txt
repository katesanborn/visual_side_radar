cmake_minimum_required(VERSION 3.0.2)
project(visual_side_radar)

find_package(catkin REQUIRED)

catkin_package()

catkin_install_python(PROGRAMS
  src/radar_display.py
  src/radar_grid_lat_long.py
  src/radar_track_simulator.py
  src/radar_angle.py
  src/radar_grid_120.py
  src/radar_angle_120.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
