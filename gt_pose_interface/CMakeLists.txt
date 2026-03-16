cmake_minimum_required(VERSION 3.0.2)
project(gt_pose_interface)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  sensor_msgs
  tf
  pcl_ros
  pcl_conversions
)

find_package(PCL REQUIRED)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

add_executable(gtPoseInterface src/gtPoseInterface.cpp)
target_link_libraries(gtPoseInterface ${catkin_LIBRARIES} ${PCL_LIBRARIES})
