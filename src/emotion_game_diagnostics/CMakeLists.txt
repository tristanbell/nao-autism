cmake_minimum_required(VERSION 2.8.3)
project(emotion_game_diagnostics)

find_package(catkin REQUIRED COMPONENTS nao_control nao_msgs roscpp std_msgs nao_autism_messages)
find_package(Boost REQUIRED COMPONENTS system)

find_package(Qt4 4.6 REQUIRED COMPONENTS QtCore QtGui)
include(${QT_USE_FILE})

set(gui_SRCS
	src/main.cpp
	src/Window.cpp
	src/NaoDiagnostics.cpp
	src/GeneralInformationBox.cpp
	src/NodeBox.cpp
	src/ExecutionControlBox.cpp
)

set(gui_HDRS
	include/Window.h
	include/NaoDiagnostics.h
	include/GeneralInformationBox.h
	include/NodeBox.h
	include/ExecutionControlBox.h
)

qt4_wrap_cpp(gui_MOCS ${gui_HDRS})

set(checker_SRCS
	src/NaoDiagnostics.cpp
	src/CheckerNode.cpp
)

set(checker_HDRS
	include/NaoDiagnostics.h
)

# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

# generate_messages(
#   DEPENDENCIES
#   nao_msgs#   std_msgs
# )

catkin_package(
   INCLUDE_DIRS include
#  LIBRARIES emotion_game_diagnostics
#  CATKIN_DEPENDS nao_control nao_msgs roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_executable(run_gui ${gui_SRCS} ${gui_MOCS})
target_link_libraries(run_gui ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${QT_LIBRARIES})

add_executable(run_checker ${checker_SRCS} ${checker_HDRS})
target_link_libraries(run_checker ${catkin_LIBRARIES} ${Boost_LIBRARIES})
