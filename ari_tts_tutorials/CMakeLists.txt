cmake_minimum_required(VERSION 2.8.3)
project(ari_tts_tutorials)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
)

catkin_package()

include_directories(SYSTEM ${catkin_INCLUDE_DIRS})
