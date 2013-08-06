/*
 * NaoMimicker.cpp
 *
 *  Created on: Aug 5, 2013
 *      Author: parallels
 */

#include <ros/ros.h>
#include <tf/tfMessage.h>
#include <tf/tf.h>
#include <nao_msgs/JointAnglesWithSpeed.h>

#include <vector>
#include <string>

ros::Publisher pub;
bool ready;

void constructAndPublishMsg(std::vector<std::string> &joints, std::vector<float> &angles)
{
	nao_msgs::JointAnglesWithSpeed jointMsg;
	jointMsg.speed = 0.5;
	jointMsg.joint_names = joints;
	jointMsg.joint_angles = angles;

	for (int i = 0; i < angles.size(); i++) {
		jointMsg.joint_angles[i] *= (M_PI / 180);
//		printf("Degrees: %f, Radians: %f\n", angles[i], jointMsg.joint_angles[i]);
	}

	pub.publish(jointMsg);
	joints.clear();
	angles.clear();
}

void tfCallback(const tf::tfMessage msg)
{
	//Allow for application to continue when message containing tf for head_1 is recieved.
	geometry_msgs::TransformStamped transform = msg.transforms[0];
	if (!ready && transform.child_frame_id.find("head_") != std::string::npos) {
		ROS_INFO("/tf is now publishing, ready to go.");
		ready = true;
		return;
	}

	float x = transform.transform.rotation.x;
	float y = transform.transform.rotation.y;
	float z = transform.transform.rotation.z;
	float w = transform.transform.rotation.w;
	tf::Quaternion quat(x, y, z, w);
	tf::Matrix3x3 mat(quat);
	double roll, pitch, yaw;

	mat.getRPY(roll, pitch, yaw);
	std::vector<std::string> joints;
	std::vector<float> angles;

	if (transform.child_frame_id.find("left_shoulder_1")
			!= std::string::npos) {
		joints.push_back("LShoulderPitch");
		joints.push_back("LShoulderRoll");

		pitch += 90;
		angles.push_back((float) pitch);
		angles.push_back((float) roll);
	}
	else if (transform.child_frame_id.find("right_shoulder_1")
			!= std::string::npos) {
		joints.push_back("RShoulderPitch");
		joints.push_back("RShoulderRoll");

		pitch += 90;
		angles.push_back((float) pitch);
		angles.push_back((float) roll);
	}
	else if (transform.child_frame_id.find("left_elbow_1")
			!= std::string::npos) {
		joints.push_back("LElbowYaw");
		joints.push_back("LElbowRoll");

		angles.push_back((float) yaw);
		angles.push_back((float) roll);
	}
	else if (transform.child_frame_id.find("right_elbow_1")
			!= std::string::npos) {
		joints.push_back("RElbowYaw");
		joints.push_back("RElbowRoll");

		angles.push_back((float) yaw);
		angles.push_back((float) roll);
	}

	constructAndPublishMsg(joints, angles);
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "mimicker");
	ros::NodeHandle nh;
	pub = nh.advertise<nao_msgs::JointAnglesWithSpeed>("/joint_angles", 10);
	ros::Subscriber sub = nh.subscribe("/tf", 20, tfCallback);

	ros::Rate rate(60);
	ROS_INFO("Waiting for a head tf to be published.");
	while (!ready) {
		ros::spinOnce();

		if (!nh.ok()) {
			ROS_INFO("Closing down.");

			exit(EXIT_FAILURE);
		}

		rate.sleep();
	}

	ROS_INFO("All set up, go!");
	ros::spin();
	return 0;
}



