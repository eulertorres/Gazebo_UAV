cmake_minimum_required(VERSION 3.0.2)
project(simulacao)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  gazebo_ros
  std_msgs
)

find_package(gazebo REQUIRED)

catkin_package(
	CATKIN_DEPENDS
	roscpp
	rospy
)

link_directories(${GAZEBO_LIBRARY_DIRS})

include_directories(
	${catkin_INCLUDE_DIRS}
	${src}
	${GAZEBO_INCLUDE_DIRS}
)

list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")