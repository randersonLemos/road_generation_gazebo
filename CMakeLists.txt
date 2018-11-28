cmake_minimum_required(VERSION 2.8.3)
project(road_generation_gazebo)

find_package(catkin REQUIRED COMPONENTS rospy gazebo_ros)

catkin_python_setup()

catkin_package(DEPENDS rospy gazebo_ros)

catkin_install_python(PROGRAMS script/road_gazebo.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
