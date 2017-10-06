cmake_minimum_required(VERSION 2.8.3)
project(road_generation_gazebo)

find_package(catkin REQUIRED COMPONENTS rospy gazebo_ros)

#include_directories(include ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS} ${SDFormat_INCLUDE_DIRS})

catkin_python_setup()

catkin_package(DEPENDS gazebo_ros)

catkin_install_python(PROGRAMS bin/main.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
