cmake_minimum_required(VERSION 2.8.3)
project(nao_msgs)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS message_generation roscpp rospy actionlib_msgs std_msgs)


#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
add_message_files(
	FILES
	Bumper.msg
	FadeRGB.msg
	JointAnglesWithSpeed.msg
	JointAngleTrajectory.msg
	TactileTouch.msg
	TorsoIMU.msg
	TorsoOdometry.msg
	WordRecognized.msg
)

add_action_files(
	FILES 
	RunBehavior.action)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
	DEPENDENCIES
	actionlib_msgs
	std_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  LIBRARIES nao_msgs
#  CATKIN_DEPENDS message_generation roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include
  ${catkin_INCLUDE_DIRS}
)
