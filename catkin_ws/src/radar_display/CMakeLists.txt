cmake_minimum_required(VERSION 3.0.2)
project(radar_display)

find_package(catkin REQUIRED)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/radar_display.py
  scripts/radar_grid_lat_long.py
  scripts/radar_track_simulator.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
