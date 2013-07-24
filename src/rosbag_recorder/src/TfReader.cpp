/*
 * TfReader.cpp
 *
 *  Created on: Jul 24, 2013
 *      Author: parallels
 */

#include <ros/ros.h>
#include <tf/tfMessage.h>
#include <geometry_msgs/TransformStamped.h>

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

void tfCallback(const tf::tfMessage msg) {
	if (msg.transforms.size() > 0) {
		geometry_msgs::TransformStamped tsn = msg.transforms[0];

		std::string str(tsn.child_frame_id);
		for (int i = 0; i < str.length(); i++) {
			char curr = str[i];

			if (curr >= '0' && curr <= '9') {
				std::string newStr(str.substr(i));

				int val = strtol(newStr.c_str(), NULL, 10);

				if (pose_map.find(val) == pose_map.end()) {
					pose_map.insert(
							std::pair<int, TempPoseData>(val, TempPoseData()));
				}

				geometry_msgs::TransformStamped* ts =
						new geometry_msgs::TransformStamped(tsn);
				TempPoseData& point = pose_map.at(val);

				//Long if statement ftw...
				if (str.find("head") != std::string::npos) {
					point.head = ts;
				} else if (str.find("neck") != std::string::npos) {
					point.neck = ts;
				} else if (str.find("torso") != std::string::npos) {
					point.torso = ts;
				} else if (str.find("left_shoulder") != std::string::npos) {
					point.left_shoulder = ts;
				} else if (str.find("left_elbow") != std::string::npos) {
					point.left_elbow = ts;
				} else if (str.find("left_hand") != std::string::npos) {
					point.left_hand = ts;
				} else if (str.find("right_shoulder") != std::string::npos) {
					point.right_shoulder = ts;
				} else if (str.find("right_elbow") != std::string::npos) {
					point.right_elbow = ts;
				} else if (str.find("right_hand") != std::string::npos) {
					point.right_hand = ts;
				} else if (str.find("left_hip") != std::string::npos) {
					point.left_hip = ts;
				} else if (str.find("left_knee") != std::string::npos) {
					point.left_knee = ts;
				} else if (str.find("left_foot") != std::string::npos) {
					point.left_foot = ts;
				} else if (str.find("right_hip") != std::string::npos) {
					point.right_hip = ts;
				} else if (str.find("right_knee") != std::string::npos) {
					point.right_knee = ts;
				} else if (str.find("right_foot") != std::string::npos) {
					point.right_foot = ts;

					if (point.valid()) {
						// Do printing here
					}

					point.clear();
				}
			}
		}
	}
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "tf_reader");

	ros::NodeHandle nh;
	ros::Subscriber sub = nh.subscribe("/tf", 15, tfCallback);

	ros::spin();

	return 0;
}

