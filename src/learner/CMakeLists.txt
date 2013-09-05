cmake_minimum_required(VERSION 2.8.3)
project(learner)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy rosbag tf std_msgs message_generation nao_autism_messages)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem date_time)
find_package(Qt4 4.6 REQUIRED COMPONENTS QtCore QtGui)
include(${QT_USE_FILE})


catkin_package(
	INCLUDE_DIRS include
	LIBRARIES ${PROJECT_NAME}
	CATKIN_DEPENDS message_runtime
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
  ${LibSVM_INCLUDE_DIRS}
)

set(learner_SRCS
		 src/classification/DataLoader.cpp
		 src/classification/DataLoaderNode.cpp
		 src/classification/KNearestNeighbour.cpp
		 src/classification/KNearestNeighbour.cpp
		 src/classification/PlainDataStore.cpp
	  	 src/classification/PoseDataPoint.cpp)
set(learner_HDRS
		 include/PoseData.h
		 include/classification/DataLoader.h
		 include/classification/DataPoint.h
		 include/classification/DataStore.h
		 include/classification/KNearestNeighbour.h
		 include/classification/DWKNearestNeighbour.h
		 include/classification/Learner.h
		 include/classification/PoseDataPoint.h
		 include/classification/PlainDataStore.h)

set(knn_node_SRCS
		src/classification/PoseDataPoint.cpp
		src/classification/KNearestNeighbour.cpp
		src/classification/DWKNearestNeighbour.cpp
		src/classification/DataLoader.cpp
		src/classification/PlainDataStore.cpp
		src/classification/KNearestNeighbourNode.cpp)
		
set(rf_node_SRCS
	src/classification/RFNode.cpp
)
		
set(svm_node_SRCS
		src/classification/SVMNode.cpp
		libsvm-3.17/svm.cpp)
		
set(svm_export_SRCS
		src/classification/PoseDataPoint.cpp
		src/classification/KNearestNeighbour.cpp
		src/classification/DWKNearestNeighbour.cpp
		src/classification/DataLoader.cpp
		src/classification/PlainDataStore.cpp
		src/classification/SvmExport.cpp)
		
set(gui_HDRS
		include/classification/accuracyTester/AccuracyTesterView.h
		include/classification/accuracyTester/AccuracyTesterModel.h)
		
set(gui_SRCS
		src/classification/accuracyTester/AccuracyTesterView.cpp
		src/classification/accuracyTester/AccuracyTesterModel.cpp)
		
qt4_wrap_cpp(gui_MOCS ${gui_HDRS})

add_executable(accuracy_checker ${gui_SRCS} ${gui_MOCS})
target_link_libraries(accuracy_checker ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${QT_LIBRARIES})

add_executable(dataLoader ${learner_SRCS} ${learner_HDRS})
target_link_libraries(dataLoader ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(svm_node ${svm_node_SRCS})
target_link_libraries(svm_node ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${LibSVM_LIBRARIES})
add_dependencies(svm_node nao_autism_messages_generate_msg_cpp)

add_executable(classification_listener src/classification/ClassificationListener.cpp)
target_link_libraries(classification_listener ${catkin_LIBRARIES})
add_dependencies(classification_listener nao_autism_messages_generate_msg_cpp)

add_executable(knn_node ${knn_node_SRCS} ${learner_HDRS})
target_link_libraries(knn_node ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(knn_node nao_autism_messages_generate_msg_cpp)

add_executable(rf_node ${rf_node_SRCS} ${learner_HDRS})
target_link_libraries(rf_node ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS})
add_dependencies(rf_node nao_autism_messages_generate_msg_cpp)

add_executable(svm_export ${svm_export_SRCS} ${learner_HDRS})
target_link_libraries(svm_export ${catkin_LIBRARIES} ${Boost_LIBRARIES})



