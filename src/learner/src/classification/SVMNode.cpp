/*
 * SVMNode.cpp
 *
 *  Created on: Jul 23, 2013
 *      Author: parallels
 */

#include <libsvm/svm.h>

#include <classification/Learner.h>
#include <classification/KNearestNeighbour.h>
#include <classification/DWKNearestNeighbour.h>
#include <classification/DataLoader.h>
#include <classification/DataStore.h>
#include <classification/PlainDataStore.h>
#include <classification/DataPoint.h>
#include <classification/PoseDataPoint.h>

#include <nao_autism_messages/PoseClassification.h>

#include <vector>
#include <map>
#include <exception>
#include <iostream>
#include <fstream>

#include <ros/ros.h>
#include <tf/tfMessage.h>
#include <geometry_msgs/TransformStamped.h>

#include <std_msgs/Int32.h>

//Method prototypes
void tfCallback(const tf::tfMessage msg);
svm_node *convertToNode(PoseData pose);

ros::Subscriber _tf_subscriber;
ros::Publisher _classification_publisher;
svm_model *_model;

class TempPoseData
{

public:
	TempPoseData()
	{
		nullAll();
	}

	geometry_msgs::TransformStamped* head;
	geometry_msgs::TransformStamped* neck;
	geometry_msgs::TransformStamped* torso;
	geometry_msgs::TransformStamped* left_shoulder;
	geometry_msgs::TransformStamped* left_elbow;
	geometry_msgs::TransformStamped* left_hand;
	geometry_msgs::TransformStamped* right_shoulder;
	geometry_msgs::TransformStamped* right_elbow;
	geometry_msgs::TransformStamped* right_hand;
	geometry_msgs::TransformStamped* left_hip;
	geometry_msgs::TransformStamped* left_knee;
	geometry_msgs::TransformStamped* left_foot;
	geometry_msgs::TransformStamped* right_hip;
	geometry_msgs::TransformStamped* right_knee;
	geometry_msgs::TransformStamped* right_foot;

	bool valid() const
	{
		return head && neck && torso && left_shoulder && left_elbow && left_hand &&
				right_shoulder && right_elbow && right_hand && left_hip && left_knee &&
				left_foot && right_hip && right_knee && right_foot;
	}

	void clear()
	{
		deleteAll();
		nullAll();
	}

private:
	inline void nullAll()
	{
		head = NULL;
		neck = NULL;
		torso = NULL;
		left_shoulder = NULL;
		left_elbow = NULL;
		left_hand = NULL;
		right_shoulder = NULL;
		right_elbow = NULL;
		right_hand = NULL;
		left_hip = NULL;
		left_knee = NULL;
		left_foot = NULL;
		right_hip = NULL;
		right_knee = NULL;
		right_foot = NULL;
	}

	inline void deleteAll()
	{
		if (head){
			delete head;
		}

		if (neck){
			delete neck;
		}

		if (torso){
			delete torso;
		}

		if (left_shoulder){
			delete left_shoulder;
		}

		if (left_elbow){
			delete left_elbow;
		}

		if (left_hand){
			delete left_hand;
		}

		if (right_shoulder){
			delete right_shoulder;
		}

		if (right_elbow){
			delete right_elbow;
		}

		if (right_hand){
			delete right_hand;
		}

		if (left_hip){
			delete left_hip;
		}

		if (left_knee){
			delete left_knee;
		}

		if (left_foot){
			delete left_foot;
		}

		if (right_hip){
			delete right_hip;
		}

		if (right_knee){
			delete right_knee;
		}

		if (right_foot){
			delete right_foot;
		}
	}
};

std::map<int, TempPoseData> pose_map;

int main(int argc, char** argv) {
	if (argc > 1) {
		ros::init(argc, argv, "svm_node");

		_model = ::svm_load_model(argv[1]);

		ROS_INFO("Created model.");

		ROS_INFO("Creating subscriber to /tf");
		ros::NodeHandle nh;

		_tf_subscriber = nh.subscribe("/tf", 15, tfCallback);

		ROS_INFO("Creating classification advertiser");
		_classification_publisher = nh.advertise<nao_autism_messages::PoseClassification>(
				"/classification", 1);

		ROS_INFO("Setup complete, spinning");
		ros::spin();

		return 0;
	} else {
		std::cout << "Usage: rosrun learner svm_node <libsvm model file>" << std::endl;
		return -1;
	}
}

void tfCallback(const tf::tfMessage msg)
{
	if (msg.transforms.size() > 0){
		geometry_msgs::TransformStamped tsn = msg.transforms[0];

		std::string str(tsn.child_frame_id);
		for (int i=0;i<str.length();i++){
			char curr = str[i];

			if (curr >= '0' && curr <= '9'){
				std::string newStr(str.substr(i));

				int val = strtol(newStr.c_str(), NULL, 10);

				if (pose_map.find(val) == pose_map.end()){
					pose_map.insert(std::pair<int, TempPoseData>(val, TempPoseData()));
				}

				geometry_msgs::TransformStamped* ts = new geometry_msgs::TransformStamped(tsn);
				TempPoseData& point = pose_map.at(val);

				//Long if statement ftw...
				if (str.find("head") != std::string::npos){
					point.head = ts;
				}else if (str.find("neck") != std::string::npos){
					point.neck = ts;
				}else if (str.find("torso") != std::string::npos){
					point.torso = ts;
				}else if (str.find("left_shoulder") != std::string::npos){
					point.left_shoulder = ts;
				}else if (str.find("left_elbow") != std::string::npos){
					point.left_elbow = ts;
				}else if (str.find("left_hand") != std::string::npos){
					point.left_hand = ts;
				}else if (str.find("right_shoulder") != std::string::npos){
					point.right_shoulder = ts;
				}else if (str.find("right_elbow") != std::string::npos){
					point.right_elbow = ts;
				}else if (str.find("right_hand") != std::string::npos){
					point.right_hand = ts;
				}else if (str.find("left_hip") != std::string::npos){
					point.left_hip = ts;
				}else if (str.find("left_knee") != std::string::npos){
					point.left_knee = ts;
				}else if (str.find("left_foot") != std::string::npos){
					point.left_foot = ts;
				}else if (str.find("right_hip") != std::string::npos){
					point.right_hip = ts;
				}else if (str.find("right_knee") != std::string::npos){
					point.right_knee = ts;
				}else if (str.find("right_foot") != std::string::npos){
					point.right_foot = ts;

					if (point.valid()){
						PoseData poseData;

						poseData.head = *point.head;
						poseData.neck = *point.neck;
						poseData.torso = *point.torso;
						poseData.left_shoulder = *point.left_shoulder;
						poseData.left_elbow = *point.left_elbow;
						poseData.left_hand = *point.left_hand;
						poseData.right_shoulder = *point.right_shoulder;
						poseData.right_elbow = *point.right_elbow;
						poseData.right_hand = *point.right_hand;
						poseData.left_hip = *point.left_hip;
						poseData.left_knee = *point.left_knee;
						poseData.left_foot = *point.left_foot;
						poseData.right_hip = *point.right_hip;
						poseData.right_knee = *point.right_knee;
						poseData.right_foot = *point.right_foot;

						//Classify point
						svm_node *node = convertToNode(poseData);
						double thisClass = ::svm_predict(_model, node);

						//Create message and send classification
						nao_autism_messages::PoseClassification pc;

						pc.user_number = val;
						pc.classification = (int) thisClass;
						pc.pose_data = poseData.getJoints();
						pc.source = "svm";

						_classification_publisher.publish(pc);
					}

					point.clear();
				}
			}
		}
	}
}

svm_node *convertToNode(PoseData data) {
	std::vector<geometry_msgs::TransformStamped> joints = data.getJoints();
	int nodesSize = joints.size()*4 + 1;
	svm_node *nodes = new svm_node[nodesSize];
	int indexCount = 1;

	for (int i = 0; i < joints.size(); i++) {
		svm_node n1, n2, n3, n4;

		n1.index = indexCount++;
		n1.value = joints[i].transform.rotation.x;
		nodes[indexCount-2] = n1;

		n2.index = indexCount++;
		n2.value = joints[i].transform.rotation.y;
		nodes[indexCount-2] = n2;

		n3.index = indexCount++;
		n3.value = joints[i].transform.rotation.z;
		nodes[indexCount-2] = n3;

		n4.index = indexCount++;
		n4.value = joints[i].transform.rotation.w;
		nodes[indexCount-2] = n4;
	}

	svm_node last;
	last.index = -1;
	nodes[nodesSize - 1] = last;

//	std::ofstream out("nao_autism", std::ios::app);
//
//	out << "+0 ";
//
//	for (int i = 0; i < nodesSize-1; i++) {
//		out << nodes[i].index << ":" << nodes[i].value << " ";
//	}
//
//	out << "\n";
//	out.close();

	return nodes;
}























