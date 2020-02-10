cmake_minimum_required(VERSION 2.8.3)
project(ari_aruco_demo)

find_package(catkin REQUIRED COMPONENTS
  aruco_ros
)


catkin_package(
)

include_directories(SYSTEM ${catkin_INCLUDE_DIRS})

