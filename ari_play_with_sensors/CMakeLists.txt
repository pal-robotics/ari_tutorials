cmake_minimum_required(VERSION 2.8.3)
project(ari_play_with_sensors)


find_package(catkin REQUIRED COMPONENTS
  rospy
)


catkin_package()


install(PROGRAMS
    scripts/current_play.py
    scripts/image_play.py
    scripts/laser_play.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

include_directories(SYSTEM ${catkin_INCLUDE_DIRS})
