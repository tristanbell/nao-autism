cmake_minimum_required(VERSION 2.8.3)
project(nao_control)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg actionlib actionlib_msgs nao_msgs)
find_package(Boost REQUIRED COMPONENTS system)

set(nao-control_SRCS src/nao_control/NaoControl.cpp)
set(nao-control_HDRS include/nao_control/NaoControl.h)

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
add_dependencies(nao_control nao_msgs_generate_msg_cpp)

install(DIRECTORY include/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
