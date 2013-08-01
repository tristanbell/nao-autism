cmake_minimum_required(VERSION 2.8.3)
project(emotion_game)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS nao_control nao_msgs roscpp std_msgs)
find_package(Boost REQUIRED COMPONENTS system)

set(start_game_SRCS
	src/main.cpp
	src/Phrase.cpp
	src/GameSettings.cpp
	src/Behavior.cpp
	src/GuessGame.cpp
	src/MimicGame.cpp
)

set(start_game_HDRS
	include/Phrase.h
	include/GameSettings.h
	include/Game.h
	include/Behavior.h
	include/MimicGame.h
	include/GuessGame.h
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

add_executable(start_game ${start_game_SRCS} ${start_game_HDRS})
target_link_libraries(start_game ${catkin_LIBRARIES} ${Boost_LIBRARIES})
