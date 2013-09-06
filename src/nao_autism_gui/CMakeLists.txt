cmake_minimum_required(VERSION 2.8.3)
project(nao_autism_gui)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs nao_control rosbag_recorder)
find_package(Qt4 4.6 REQUIRED COMPONENTS QtCore QtGui)
include(${QT_USE_FILE})

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${QT_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

set(nao-autism_SRCS
	src/main.cpp
	src/NaoBehavior.cpp
	src/NaoSpeech.cpp
	src/NaoSpeechData.cpp
	src/NaoAutismWindow.cpp
	src/NaoGuessBox.cpp
	src/NaoMimicBox.cpp
	src/GenericControlBox.cpp)

set(nao-autism_HDRS
	include/NaoBehavior.h
	include/NaoSpeech.h
	include/NaoSpeechData.h
	include/nao_gui/NaoAutismWindow.h
	include/nao_gui/NaoGuessBox.h
	include/nao_gui/NaoMimicBox.h
	include/nao_gui/GenericControlBox.h)

qt4_wrap_cpp(nao-autism_MOCS ${nao-autism_HDRS})

catkin_package(
   INCLUDE_DIRS include
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_executable(run_autism_gui ${nao-autism_SRCS} ${nao-autism_MOCS})
target_link_libraries(run_autism_gui ${QT_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(run_autism_gui rosbag_recorder_gencpp)
