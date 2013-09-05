cmake_minimum_required(VERSION 2.8.3)
project(rosbag_recorder)

find_package(catkin REQUIRED COMPONENTS roscpp rospy rosbag tf std_msgs message_generation)
find_package(Boost REQUIRED)

add_message_files(
  FILES
  Record.msg
)

 generate_messages(
   DEPENDENCIES
   std_msgs
 )

catkin_package(
	INCLUDE_DIRS include
	LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS message_runtime
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_executable(run_rosbag_recorder src/recorder.cpp)
target_link_libraries(run_rosbag_recorder ${catkin_LIBRARIES})
add_dependencies(run_rosbag_recorder nao_autism_messages_generate_message_cpp)

add_library(rosbag_recorder src/recorder.cpp include/recorder.h)
target_link_libraries(rosbag_recorder ${catkin_LIBRARIES})

install(DIRECTORY include/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

