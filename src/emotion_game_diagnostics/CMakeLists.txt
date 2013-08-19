cmake_minimum_required(VERSION 2.8.3)
project(emotion_game_diagnostics)

find_package(catkin REQUIRED COMPONENTS nao_control nao_msgs roscpp std_msgs)
find_package(Boost REQUIRED COMPONENTS system)

find_package(Qt4 4.6 REQUIRED COMPONENTS QtCore QtGui)
include(${QT_USE_FILE})

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
