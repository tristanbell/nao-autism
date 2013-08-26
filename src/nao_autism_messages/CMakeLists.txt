cmake_minimum_required(VERSION 2.8.3)
project(nao_autism_messages)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS actionlib_msgs std_msgs geometry_msgs)


#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
add_message_files(
  	FILES
   	PoseClassification.msg
  	Record.msg
  	ExecutionStatus.msg
  	SetExecutionStatus.msg
)

add_action_files(
	FILES 
	Behavior.action)

## Generate added messages and services with any dependencies listed here
generate_messages(
	DEPENDENCIES
	actionlib_msgs
	std_msgs
	geometry_msgs
)

catkin_package(
)

