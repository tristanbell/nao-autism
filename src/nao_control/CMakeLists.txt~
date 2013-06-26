cmake_minimum_required(VERSION 2.8.3)
project(nao_control)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg actionlib actionlib_msgs)
find_package(Boost REQUIRED COMPONENTS system)

#find_package(catkin REQUIRED genmsg actionlib_msgs)
add_action_files(DIRECTORY action FILES Behavior.action)
generate_messages(DEPENDENCIES actionlib_msgs std_msgs)

set(nao-control_SRCS src/NaoControl.cpp)
set(nao-control_HDRS include/NaoControl.h)

## Declare a catkin package
catkin_package(
	INCLUDE_DIRS include
	LIBRARIES ${PROJECT_NAME}
	CATKIN_DEPENDS  actionlib_msgs
)

## Build NaoControl
include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_library(nao_control ${nao-control_SRCS} ${nao-control_HDRS})
target_link_libraries(nao_control ${catkin_LIBRARIES})

install(DIRECTORY include
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
