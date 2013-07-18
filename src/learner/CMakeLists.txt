cmake_minimum_required(VERSION 2.8.3)
project(learner)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy rosbag tf std_msgs message_generation)
find_package(Boost REQUIRED COMPONENTS system filesystem serialization date_time)

add_message_files(
	FILES
	PoseClassification.msg
)

generate_messages(
	DEPENDENCIES
	std_msgs)

catkin_package(
	INCLUDE_DIRS include
	LIBRARIES ${PROJECT_NAME}
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
)

set(learner_SRCS
		 src/classification/DataLoader.cpp
		 src/classification/KNearestNeighbour.cpp
		 src/classification/PlainDataStore.cpp
	  	 src/classification/PoseDataPoint.cpp)
set(learner_HDRS
		 include/PoseData.h
		 include/classification/DataLoader.h
		 include/classification/DataPoint.h
		 include/classification/DataStore.h
		 include/classification/KNearestNeighbour.h
		 include/classification/Learner.h
		 include/classification/PoseDataPoint.h
		 include/classification/PlainDataStore.h)

set(knn_node_SRCS
		src/classification/PoseDataPoint.cpp
		src/classification/KNearestNeighbour.cpp
		src/classification/DataLoader.cpp
		src/classification/PlainDataStore.cpp
		src/classification/KNearestNeighbourNode.cpp)

#add_executable(dataLoader ${learner_SRCS} ${learner_HDRS})
#target_link_libraries(dataLoader ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(knn_node ${knn_node_SRCS} ${learner_HDRS})
target_link_libraries(knn_node ${catkin_LIBRARIES} ${Boost_LIBRARIES})

#add_executable(PlainDataStoreTest ${learner_SRCS} ${learner_HDRS})
#target_link_libraries(PlainDataStoreTest ${catkin_LIBRARIES})



