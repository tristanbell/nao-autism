cmake_minimum_required(VERSION 2.8.3)
project(emotion_game)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS nao_control nao_msgs roscpp std_msgs)
find_package(Boost REQUIRED COMPONENTS system)

set(start_game_SRCS
	src/jsoncpp.cpp ## JSON C++ Library
	src/main.cpp
	src/Phrase.cpp
	src/GameSettings.cpp
	src/Behavior.cpp
	src/Game.cpp
	src/GuessGame.cpp
	src/MimicGame.cpp
)

set(start_game_HDRS
	include/json/json.h ## JSON C++ Library
	#include/json/json-forwards.h ## JSON C++ Library
	include/Phrase.h
	include/GameSettings.h
	include/Game.h
	include/Behavior.h
	include/MimicGame.h
	include/GuessGame.h
	include/Keys.h
)

set(gen_json_SRCS
	src/GenerateJSONFile.cpp

	src/jsoncpp.cpp
)

set(gen_json_HDRS
	include/json/json.h
	include/Keys.h
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
add_dependencies(start_game nao_msgs_generate_msg_cpp)

add_executable(gen_json ${gen_json_SRCS} ${gen_json_HDRS})
target_link_libraries(gen_json ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(mimicker src/NaoMimicker.cpp)
target_link_libraries(mimicker ${catkin_LIBRARIES})
