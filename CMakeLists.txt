cmake_minimum_required(VERSION 2.8.3)
project(leg_chaser)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
	std_msgs
	)

find_package(cmake_modules REQUIRED)
find_package(OpenCV REQUIRED)
#find_package(Eigen REQUIRED)

catkin_package(
	  INCLUDE_DIRS include
	  LIBRARIES leg_chaser
	  CATKIN_DEPENDS roscpp rospy std_msgs
	  DEPENDS system_lib Eigen
	)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
	#${Eigen_INCLUDE_DIRS}
)

add_library(leg_detection
	src/leg_detection.cpp
	)

add_executable(leg_detection_node
	src/leg_detection_node.cpp
	)

add_executable(leg_chaser_node
	src/leg_chaser_node.cpp
	)

target_link_libraries(leg_detection_node
	leg_detection
	${catkin_LIBRARIES}
	${OpenCV_LIBRARIES}
	#${Eigen_LIBRARIES}
	)

target_link_libraries(leg_chaser_node
	leg_detection
	${catkin_LIBRARIES}
	${OpenCV_LIBRARIES}
	#${Eigen_LIBRARIES}
	)

