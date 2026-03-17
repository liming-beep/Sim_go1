cmake_minimum_required(VERSION 3.0.2)
project(water_plant_inspection)

find_package(catkin REQUIRED)

catkin_package()

catkin_install_python(PROGRAMS scripts/inspection_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
