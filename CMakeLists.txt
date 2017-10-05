cmake_minimum_required(VERSION 2.8.3)
project(road_generation_gazebo)

find_package(catkin REQUIRED COMPONENTS rospy)

catkin_python_setup()

catkin_package()

catkin_install_python(PROGRAMS bin/main.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
