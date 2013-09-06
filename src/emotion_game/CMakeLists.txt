cmake_minimum_required(VERSION 2.8.3)
project(emotion_game)

find_package(catkin REQUIRED COMPONENTS nao_autism_messages nao_control nao_msgs roscpp std_msgs visualization_msgs tf)
find_package(Boost REQUIRED COMPONENTS system)

find_package(Qt4 4.6 REQUIRED COMPONENTS QtCore QtGui)
include(${QT_USE_FILE})

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
	src/GenerateJSONNode.cpp

	src/jsoncpp.cpp
)

set(gen_json_HDRS
	include/json/json.h
	include/GenerateJSONFile.h
	include/Keys.h
)

catkin_package(
  INCLUDE_DIRS include
)

include_directories(
	include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS}
)

add_executable(start_game ${start_game_SRCS} ${start_game_HDRS})
target_link_libraries(start_game ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${QT_LIBRARIES})
add_dependencies(start_game nao_msgs_generate_msg_cpp)

add_executable(gen_json ${gen_json_SRCS} ${gen_json_HDRS})
target_link_libraries(gen_json ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(mimicker src/NaoMimicker.cpp)
target_link_libraries(mimicker ${catkin_LIBRARIES})

add_executable(measurer src/TfMeasure.cpp)
target_link_libraries(measurer ${catkin_LIBRARIES})

set(INSTALL_TARGETS
	start_game
	gen_json
	mimicker
)

install(TARGETS ${INSTALL_TARGETS}
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
