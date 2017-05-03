cmake_minimum_required(VERSION 2.8.3)
project(joy_control_python)

find_package(Boost REQUIRED thread date_time system filesystem)

find_package(PkgConfig REQUIRED)


find_package(catkin REQUIRED COMPONENTS
  moveit_ros_planning_interface
  moveit_ros_robot_interaction
  moveit_ros_warehouse
  moveit_ros_perception
  cmake_modules
  eigen_conversions
  geometric_shapes
  interactive_markers
  class_loader
  rviz
  tf
  roscpp
  rosconsole
  object_recognition_msgs
)
find_package(Eigen3 REQUIRED)

catkin_package(
  #LIBRARIES

  #INCLUDE_DIRS

  #CATKIN_DEPENDS

    )

catkin_install_python(PROGRAMS scripts/joy_control_start.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
catkin_python_setup()


