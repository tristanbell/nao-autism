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
#include <visualization_msgs/Marker.h>

#include <vector>
#include <string>

#define ORIGINAL_FRAME "/openni_depth_frame"
#define REMAP_FRAME "/tf_remap"
#define CONTROL_DIST 0.4

enum RobotMovements {
	FORWARD,
	BACKWARD,
	LEFT,
	RIGHT,
	STOPX,
	STOPY
};

/*
 *  Represents the bounding box around the user which allows
 *  the user to control the Nao's walking movements.
 */
struct ControlBox {
	double front, back, left, right;

	ControlBox(geometry_msgs::Vector3 userPosition)
	{
		front = userPosition.x - CONTROL_DIST;
		back = userPosition.x + CONTROL_DIST;
		left = userPosition.y - CONTROL_DIST;
		right = userPosition.y + CONTROL_DIST;
	}
};

ControlBox *_controlBox;
RobotMovements _leftXMotionControl;
RobotMovements _leftYMotionControl;
RobotMovements _rightXMotionControl;
RobotMovements _rightYMotionControl;
ros::Publisher _arms_pub;
ros::Publisher _walk_pub;
ros::Publisher _box_pub;
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

void init(geometry_msgs::TransformStamped initTransform) {
	geometry_msgs::Vector3 userPosition = initTransform.transform.translation;
	_controlBox = new ControlBox(userPosition);
	_leftXMotionControl = STOPX;
	_rightXMotionControl = STOPX;
	_leftYMotionControl = STOPY;
	_rightYMotionControl = STOPY;

	printf("User origin: (%f, %f), box: (%f, %f, %f, %f)\n", userPosition.x,
			userPosition.z, _controlBox->front, _controlBox->back,
			_controlBox->left, _controlBox->right);

	/*
	ros::Rate r(1);
	while (ros::ok()) {
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "/openni_depth_frame";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "basic_shapes";
    marker.id = 0;

    marker.type = visualization_msgs::Marker::CUBE;

    // Set the marker action.  Options are ADD and DELETE
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = _controlBox->back;
    marker.pose.position.y = _controlBox->left;
    marker.pose.position.z = userPosition.z;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = _controlBox->right - _controlBox->left;
    marker.scale.y = _controlBox->back - _controlBox->front;
    marker.scale.z = 0.1;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    // Publish the marker
    _box_pub.publish(marker);

	r.sleep();
	}
	*/
}

void checkTfBroadcasting(geometry_msgs::TransformStamped transform)
{
	if (!ready
			&& transform.header.frame_id == ORIGINAL_FRAME
			&& transform.child_frame_id.find("left_foot") != std::string::npos) {
		init(transform);

		ROS_INFO("/tf is now publishing, ready to go.");
		ready = true;
	}
}

void constructAndPublishMsg(std::vector<std::string> &joints, std::vector<float> &angles)
{
	nao_msgs::JointAnglesWithSpeed jointMsg;
	jointMsg.speed = 0.25;
	jointMsg.joint_names = joints;
	jointMsg.joint_angles = angles;

	_arms_pub.publish(jointMsg);
}

void constructAndPublishMovement(void) {
	geometry_msgs::Twist msg;

//	std::cout << "Movement: ";

	if (_leftXMotionControl == FORWARD || _rightXMotionControl == FORWARD) {
		std::cout << "forward" << std::endl;
		msg.linear.x = 1;
	} else if (_leftXMotionControl == BACKWARD || _rightXMotionControl == BACKWARD) {
		std::cout << "backward" << std::endl;
		msg.linear.x = -1;
	}
	if (_leftXMotionControl == STOPX && _rightXMotionControl == STOPX) {
		std::cout << "stopX" << std::endl;
		msg.linear.x = 0;
	}

	// Only move left or right if not already moving forward or back
	if (msg.linear.x == 0) {
		if (_leftYMotionControl == LEFT) {
//			std::cout << "left" << std::endl;
			msg.linear.y = 1;
		} else if (_rightYMotionControl == RIGHT) {
//			std::cout << "right" << std::endl;
			msg.linear.y = -1;
		}

		if (_leftYMotionControl == STOPY && _rightYMotionControl == STOPY) {
//			std::cout << "stopY" << std::endl;
			msg.linear.y = 0;
		}
	} else {
		msg.linear.y = 0;
	}

//	std::cout << std::endl;

	_walk_pub.publish(msg);
}

void calculateLElbowRoll(void)
{
	if (_leftElbow && _leftHand && _leftShoulder) {
		tf::Vector3 elbowToHand;
		tf::Vector3 shoulderToElbow;

		float etohLength = _leftElbow->distance(*_leftHand);
		float stoeLength = _leftShoulder->distance(*_leftElbow);

		// Find distances and normalize (to between 0 and 1)
		elbowToHand.setX((_leftElbow->x() - _leftHand->x()) / etohLength);
		elbowToHand.setY((_leftElbow->y() - _leftHand->y()) / etohLength);
		elbowToHand.setZ((_leftElbow->z() - _leftHand->z()) / etohLength);

		shoulderToElbow.setX((_leftShoulder->x() - _leftElbow->x()) / stoeLength);
		shoulderToElbow.setY((_leftShoulder->y() - _leftElbow->y()) / stoeLength);
		shoulderToElbow.setZ((_leftShoulder->z() - _leftElbow->z()) / stoeLength);

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

		float theta = elbowToHand.angle(shoulderToElbow);

//		printf("Right elbow roll -- tf: %f, calculated: %f      \r", _rightElbowRoll, theta);
//		std::flush(std::cout);

		_rightElbowRoll = theta;
	}
}

void tfCallback(const tf::tfMessage msg)
{
	bool toPublish = false;
	geometry_msgs::TransformStamped transform = msg.transforms[0];

	// Only allow application to continue when message containing tf for head joint is received.
	checkTfBroadcasting(transform);

	if (ready) {
		geometry_msgs::Vector3 translation = transform.transform.translation;

		// Move arms
		if (transform.header.frame_id == REMAP_FRAME) {
			// Initialise roll, pitch and yaw for the current joint
			float x = transform.transform.rotation.x;
			float y = transform.transform.rotation.y;
			float z = transform.transform.rotation.z;
			float w = transform.transform.rotation.w;
			tf::Vector3 origin(translation.x, translation.y, translation.z);
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
				roll = -roll - (M_PI/4);

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

	//			printf("Right shoulder roll: %f, pitch: %f      \r", roll, pitch);
	//			std::flush(std::cout);
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

			// Elbow roll must be calculated manually, for some reason
			calculateLElbowRoll();
			calculateRElbowRoll();

			if (toPublish) {
				constructAndPublishMsg(joints, angles);
			}
		}
		// Control walking
		if (transform.header.frame_id == ORIGINAL_FRAME) {
			std::string child = transform.child_frame_id;
			if (child == "/left_foot_1") {
//				printf("Left foot: (%f, %f, %f)\r", translation.x, translation.y, translation.z);
//				std::flush(std::cout);

				geometry_msgs::Twist msg;

				if (translation.x < _controlBox->front) {
					std::cout << "Left forward" << std::endl;
					_leftXMotionControl = FORWARD;
				}
				else if (translation.x > _controlBox->back) {
					std::cout << "Left back" << std::endl;
					_leftXMotionControl = BACKWARD;
				}
				else {
					_leftXMotionControl = STOPX;
				}

				if (translation.y > _controlBox->right) {
					_leftYMotionControl = LEFT;
				}
				else {
					_leftYMotionControl = STOPX;
				}
			}
			if (child == "/right_foot_1") {
//				printf("Left foot: (%f, %f)\r", translation.x, translation.z);
//				std::flush(std::cout);

				if (translation.x < _controlBox->front) {
					std::cout << "Right forward" << std::endl;
					_rightXMotionControl = FORWARD;
				}
				else if (translation.x > _controlBox->back) {
					std::cout << "Right back" << std::endl;
					_rightXMotionControl = BACKWARD;
				}
				else {
					_rightXMotionControl = STOPX;
				}

				if (translation.y < _controlBox->left) {
					_rightYMotionControl = RIGHT;
				}
				else {
					_rightYMotionControl = STOPY;
				}

				constructAndPublishMovement();
			}
		}
	}
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "mimicker");
	ros::NodeHandle nh;
	_arms_pub = nh.advertise<nao_msgs::JointAnglesWithSpeed>("/joint_angles", 50);
	_walk_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 2);
	_box_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);
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



