cmake_minimum_required(VERSION 2.8.3)
project(emotion_game)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS nao_control roscpp std_msgs)
find_package(Boost REQUIRED COMPONENTS system)

set(test_SRCS
	src/main.cpp
	src/Phrase.cpp
)

set(test_HDRS
	include/Phrase.h
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES emotion_game
#  CATKIN_DEPENDS nao_control roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
	include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS}
)

add_executable(test ${test_SRCS} ${test_HDRS})
target_link_libraries(test ${catkin_LIBRARIES} ${Boost_LIBRARIES})
