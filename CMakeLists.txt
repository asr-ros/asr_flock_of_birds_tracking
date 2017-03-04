cmake_minimum_required(VERSION 2.8.3)
project(asr_flock_of_birds_tracking)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS 
	roscpp
	asr_msgs
	std_msgs
	sensor_msgs
	geometry_msgs
	tf
)
find_package(OpenCV REQUIRED )

catkin_package(
	CATKIN_DEPENDS roscpp asr_msgs std_msgs sensor_msgs geometry_msgs tf
	DEPENDS
)

include_directories(
	${PROJECT_SOURCE_DIR}/include
	${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}
	src/PTUTracker.cpp
)


# dependcy to the message generation of asr_msgs to ensure 
# the message header files exist before trying to compile this package
add_dependencies(${PROJECT_NAME} asr_msgs_generate_messages_cpp)


target_link_libraries(${PROJECT_NAME}
	${catkin_LIBRARIES}
)
