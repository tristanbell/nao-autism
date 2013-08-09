/*
 * NaoMimicker.cpp
 *  Created on: Aug 5, 2013
 *      Author: Tristan Bell
 *
 * Notes on joint correspondence (minus denotes inverse):
 * 		Kinect Data			Nao
 * 		--------------		-------------
 * 		LShoudler Roll		LShoulder Roll
 * 		LShoudler -Yaw		LShoulder Pitch
 * 		LElbow -Pitch		LElbow Yaw
 * 		LElbow -Yaw			LElbow Roll
 *
 * 		RShoudler -Roll		RShoulder Roll
 * 		RShoudler Yaw		RShoulder Pitch
 * 		RElbow -Pitch		RElbow Yaw
 * 		RElbow -Yaw			RElbow Roll
 *
 *
 * Joint name		Motion									Range (degrees)		Range (radians)
 * --------------	-------------------------------------	---------------		---------------
 * LShoulderPitch	Left shoulder joint front and back (Y)	-119.5 to 119.5		-2.0857 to 2.0857
 * LShoulderRoll	Left shoulder joint right and left (Z)	-18 to 76			-0.3142 to 1.3265
 * LElbowYaw		Left shoulder joint twist (X)			-119.5 to 119.5		-2.0857 to 2.0857
 * LElbowRoll		Left elbow joint (Z)					-88.5 to -2			-1.5446 to -0.0349
 *
 * RShoulderPitch	Right shoulder joint front and back (Y)	-119.5 to 119.5		-2.0857 to 2.0857
 * RShoulderRoll	Right shoulder joint right and left (Z)	-76 to 18			-1.3265 to 0.3142
 * RElbowYaw		Right shoulder joint twist (X)			-119.5 to 119.5		-2.0857 to 2.0857
 * RElbowRoll		Right elbow joint (Z)					2 to 88.5			0.0349 to 1.5446
 */

#include <ros/ros.h>
#include <tf/tfMessage.h>
#include <tf/tf.h>
#include <nao_msgs/JointAnglesWithSpeed.h>

#include <vector>
#include <string>

ros::Publisher pub;
bool ready;

// Positions of joints to calculate elbow roll
tf::Vector3 *_leftHand;
tf::Vector3 *_leftElbow;
tf::Vector3 *_leftShoulder;
tf::Vector3 *_rightHand;
tf::Vector3 *_rightElbow;
tf::Vector3 *_rightShoulder;

float _leftElbowRoll = 0.0;
float _rightElbowRoll = 0.0;

void constructAndPublishMsg(std::vector<std::string> &joints, std::vector<float> &angles)
{
	nao_msgs::JointAnglesWithSpeed jointMsg;
	jointMsg.speed = 0.25;
	jointMsg.joint_names = joints;
	jointMsg.joint_angles = angles;

	pub.publish(jointMsg);
}

void calculateLElbowRoll(void)
{
	if (_leftElbow && _leftHand && _leftShoulder) {
		tf::Vector3 elbowToHand;
		tf::Vector3 shoulderToElbow;

		float etohLength = _leftElbow->distance(*_leftHand);
		float stoeLength = _leftShoulder->distance(*_leftElbow);

//			// Find distances and normalize (to between 0 and 1)
		elbowToHand.setX((_leftElbow->x() - _leftHand->x()) / etohLength);
		elbowToHand.setY((_leftElbow->y() - _leftHand->y()) / etohLength);
		elbowToHand.setZ((_leftElbow->z() - _leftHand->z()) / etohLength);

		shoulderToElbow.setX((_leftShoulder->x() - _leftElbow->x()) / stoeLength);
		shoulderToElbow.setY((_leftShoulder->y() - _leftElbow->y()) / stoeLength);
		shoulderToElbow.setZ((_leftShoulder->z() - _leftElbow->z()) / stoeLength);

//			elbowToHand.setX((_leftElbow->x() + _leftHand->x()));
//			elbowToHand.setY((_leftElbow->y() + _leftHand->y()));
//			elbowToHand.setZ((_leftElbow->z() + _leftHand->z()));

//			shoulderToElbow.setX((_leftShoulder->x() + _leftElbow->x()));
//			shoulderToElbow.setY((_leftShoulder->y() + _leftElbow->y()));
//			shoulderToElbow.setZ((_leftShoulder->z() + _leftElbow->z()));

		float theta = elbowToHand.angle(shoulderToElbow);

//		printf("Left elbow roll -- tf: %f, calculated: %f      \r", _leftElbowRoll, -theta);
//		std::flush(std::cout);

		_leftElbowRoll = -theta;
	}
}

void calculateRElbowRoll(void)
{
	if (_rightElbow && _rightHand && _rightShoulder) {
		tf::Vector3 elbowToHand;
		tf::Vector3 shoulderToElbow;

		float etohLength = _rightElbow->distance(*_rightHand);
		float stoeLength = _rightShoulder->distance(*_rightElbow);

		// Find distances and normalize (to between 0 and 1)
		elbowToHand.setX((_rightElbow->x() - _rightHand->x()) / etohLength);
		elbowToHand.setY((_rightElbow->y() - _rightHand->y()) / etohLength);
		elbowToHand.setZ((_rightElbow->z() - _rightHand->z()) / etohLength);

		shoulderToElbow.setX((_rightShoulder->x() - _rightElbow->x()) / stoeLength);
		shoulderToElbow.setY((_rightShoulder->y() - _rightElbow->y()) / stoeLength);
		shoulderToElbow.setZ((_rightShoulder->z() - _rightElbow->z()) / stoeLength);

//		_elbowToHand.setX((_rightElbow->x() + _rightHand->x()));
//		_elbowToHand.setY((_rightElbow->y() + _rightHand->y()));
//		_elbowToHand.setZ((_rightElbow->z() + _rightHand->z()));

//		_shoulderToElbow.setX((_rightShoulder->x() + _rightElbow->x()));
//		_shoulderToElbow.setY((_rightShoulder->y() + _rightElbow->y()));
//		_shoulderToElbow.setZ((_rightShoulder->z() + _rightElbow->z()));

		float theta = elbowToHand.angle(shoulderToElbow);

//		printf("Right elbow roll -- tf: %f, calculated: %f      \r", _rightElbowRoll, theta);
//		std::flush(std::cout);

		_rightElbowRoll = theta;
	}
}

void tfCallback(const tf::tfMessage msg)
{
	bool toPublish = false;

	//Allow application to continue when message containing tf for head_1 is received.
	geometry_msgs::TransformStamped transform = msg.transforms[0];
	if (!ready && transform.child_frame_id.find("head_") != std::string::npos) {
		ROS_INFO("/tf is now publishing, ready to go.");
		ready = true;
		return;
	}

	if (transform.header.frame_id == "/tf_remap") {
		float x = transform.transform.rotation.x;
		float y = transform.transform.rotation.y;
		float z = transform.transform.rotation.z;
		float w = transform.transform.rotation.w;
		geometry_msgs::Vector3 o = transform.transform.translation;
		tf::Vector3 origin(o.x, o.y, o.z);
		tf::Quaternion quat(x, y, z, w);
		tf::Matrix3x3 mat(quat);

		// Roll: X (Red), Pitch: Y (Green), Yaw: Z (Blue)
		double roll, pitch, yaw;

		mat.getRPY(roll, pitch, yaw);
		std::vector<std::string> joints;
		std::vector<float> angles;

		if (transform.child_frame_id.find("left_shoulder_1")
				!= std::string::npos) {
			_leftShoulder = new tf::Vector3(origin);
			pitch = -yaw;

			if (pitch > -2.0857 && pitch < 2.0857) {
				joints.push_back("LShoulderPitch");
				angles.push_back((float) pitch);
				toPublish = true;
			}
			if (roll > -0.3142 && roll < 1.3265) {
				joints.push_back("LShoulderRoll");
				angles.push_back((float) roll);
				toPublish = true;
			}
//			printf("Left shoulder roll: %f, pitch: %f      \r", roll, pitch);
//			std::flush(std::cout);
		}
		if (transform.child_frame_id.find("right_shoulder_1")
				!= std::string::npos) {
			_rightShoulder = new tf::Vector3(origin);
			pitch = yaw;
			roll = roll - (M_PI/2);

			if (pitch > -2.0857 && pitch < 2.0857) {
				joints.push_back("RShoulderPitch");
				angles.push_back((float) pitch);
				toPublish = true;
			}
			if (roll > -1.3265 && roll < 0.3142) {
				joints.push_back("RShoulderRoll");
				angles.push_back((float) roll);
				toPublish = true;
			}

			printf("Right shoulder roll: %f, pitch: %f      \r", roll, pitch);
			std::flush(std::cout);
		}
		if (transform.child_frame_id.find("left_elbow_1")
				!= std::string::npos) {
			_leftElbow = new tf::Vector3(origin);
			yaw = -pitch;
			roll = _leftElbowRoll;

			if (yaw > -2.0857 && yaw < 2.0857) {
				joints.push_back("LElbowYaw");
				angles.push_back((float) yaw);
				toPublish = true;
			}
			if (roll > -1.5446 && roll < -0.0349) {
				joints.push_back("LElbowRoll");
				angles.push_back((float) roll);
				toPublish = true;
			}
		}
		if (transform.child_frame_id.find("right_elbow_1")
				!= std::string::npos) {
			_rightElbow = new tf::Vector3(origin);
			yaw = pitch;
			roll = _rightElbowRoll;

			if (yaw > -2.0857 && yaw < 2.0857) {
				joints.push_back("RElbowYaw");
				angles.push_back((float) pitch);
				toPublish = true;
			}
			if (roll > 0.0349 && roll < 1.5446) {
				joints.push_back("RElbowRoll");
				angles.push_back((float) roll);
				toPublish = true;
			}
		}
		if (transform.child_frame_id.find("left_hand_1")
				!= std::string::npos) {
			_leftHand = new tf::Vector3(origin);
		}
		if (transform.child_frame_id.find("right_hand_1")
				!= std::string::npos) {
			_rightHand = new tf::Vector3(origin);
		}

		calculateLElbowRoll();
		calculateRElbowRoll();

		if (toPublish) {
			constructAndPublishMsg(joints, angles);
		}
	}
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "mimicker");
	ros::NodeHandle nh;
	pub = nh.advertise<nao_msgs::JointAnglesWithSpeed>("/joint_angles", 50);
	ros::Subscriber sub = nh.subscribe("/tf", 50, tfCallback);

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

	std::cout << std::endl;
	return 0;
}



