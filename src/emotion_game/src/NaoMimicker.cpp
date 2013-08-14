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

// ROS and relevant messages
#include <ros/ros.h>
#include <tf/tfMessage.h>
#include <tf/tf.h>
#include <nao_msgs/JointAnglesWithSpeed.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/String.h>

#include <vector>
#include <string>

#define ORIGINAL_FRAME "/openni_depth_frame"
#define REMAP_FRAME "/tf_remap"
#define CONTROL_DIST 0.3

/*
 * Used to determine the direction the Nao should walk in,
 * according to the positions of limbs.
 */
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

// For initial setup, determining which user will control the Nao
std::string _userNumber = "";
ros::Publisher _user_pub;
// Links user numbers to hand and head positions
std::map< std::string, std::pair<geometry_msgs::Vector3, geometry_msgs::Vector3> > _userDistanceMap;

// For controlling the Nao
ControlBox *_controlBox;
RobotMovements _leftXMotionControl;
RobotMovements _leftYMotionControl;
RobotMovements _rightXMotionControl;
RobotMovements _rightYMotionControl;
ros::Publisher _arms_pub;
ros::Publisher _walk_pub;
bool broadcasting;
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

// Torso rotation used to turn the robot
double _initTorsoRotation, _torsoRotation = 0.0;
#define ROTATION_BOUNDARY 0.1

void init(geometry_msgs::TransformStamped initTransform) {
	// Initialise torso rotation
	float x = initTransform.transform.rotation.x;
	float y = initTransform.transform.rotation.y;
	float z = initTransform.transform.rotation.z;
	float w = initTransform.transform.rotation.w;
	tf::Quaternion quat(x, y, z, w);
	tf::Matrix3x3 mat(quat);
	double roll, pitch, yaw;
	mat.getRPY(roll, pitch, yaw);
	_initTorsoRotation = _torsoRotation = pitch;
	ROS_INFO("Initial rotation: %f", _initTorsoRotation);

	geometry_msgs::Vector3 userPosition = initTransform.transform.translation;
	_controlBox = new ControlBox(userPosition);
	_leftXMotionControl = STOPX;
	_rightXMotionControl = STOPX;
	_leftYMotionControl = STOPY;
	_rightYMotionControl = STOPY;

	printf("User origin: (%f, %f), box: (%f, %f, %f, %f)\n", userPosition.x,
			userPosition.z, _controlBox->front, _controlBox->back,
			_controlBox->left, _controlBox->right);
}

void checkTfBroadcasting(geometry_msgs::TransformStamped transform)
{
	if (!broadcasting
			&& transform.header.frame_id == ORIGINAL_FRAME
			&& transform.child_frame_id.find("left_foot") != std::string::npos) {
		ROS_INFO("/tf is now publishing, ready to go.");
		broadcasting = true;
	}
}

void constructAndPublishMsg(std::vector<std::string> &joints, std::vector<float> &angles)
{
	nao_msgs::JointAnglesWithSpeed jointMsg;
	jointMsg.speed = 0.28;
	jointMsg.joint_names = joints;
	jointMsg.joint_angles = angles;

	_arms_pub.publish(jointMsg);
}

void constructAndPublishMovement(void) {
	geometry_msgs::Twist msg;

	std::cout << "Movement: ";

	if (_leftXMotionControl == FORWARD || _rightXMotionControl == FORWARD) {
		std::cout << "forward  " << "\r";
		msg.linear.x = 1;
	} else if (_leftXMotionControl == BACKWARD || _rightXMotionControl == BACKWARD) {
		std::cout << "backward " << "\r";
		msg.linear.x = -1;
	}
	if (_leftXMotionControl == STOPX && _rightXMotionControl == STOPX) {
		std::cout << "stop    " << "\r";
		msg.linear.x = 0;
	}

	// Only move left or right if not already moving forward or back
	if (msg.linear.x == 0) {
		if (_leftYMotionControl == LEFT) {
			std::cout << "left     " << "\r";
			msg.linear.y = 1;
		} else if (_rightYMotionControl == RIGHT) {
			std::cout << "right    " << "\r";
			msg.linear.y = -1;
		}

		if (_leftYMotionControl == STOPY && _rightYMotionControl == STOPY) {
			std::cout << "stopY" << std::endl;
			msg.linear.y = 0;
		}
	} else {
		msg.linear.y = 0;
	}

//	std::cout << "Rotation: " << _torsoRotation << "         \r";

	if (_torsoRotation > _initTorsoRotation + ROTATION_BOUNDARY) {
		// Turn right
		std::cout << "Turn right      " << "\r";
		msg.angular.z = 1;
	}
	else if (_torsoRotation < _initTorsoRotation - ROTATION_BOUNDARY) {
		// Turn left
		std::cout << "Turn left       " << "\r";
		msg.angular.z = -1;
	}

	std::flush(std::cout);

	_walk_pub.publish(msg);
}

// TODO: make left and right elbow calculations into one method,
// 		 taking elbow to calculate as a parameter
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

void moveLegs(geometry_msgs::TransformStamped& transform,
		geometry_msgs::Vector3& translation)
{
	std::string child = transform.child_frame_id;
	if (child == "/left_foot" + _userNumber) {
		geometry_msgs::Twist msg;
		if (translation.x < _controlBox->front) {
			_leftXMotionControl = FORWARD;
		} else if (translation.x > _controlBox->back) {
			_leftXMotionControl = BACKWARD;
		} else {
			_leftXMotionControl = STOPX;
		}

		if (translation.y > _controlBox->right) {
			_leftYMotionControl = LEFT;
		} else {
			_leftYMotionControl = STOPX;
		}
	}

	if (child == "/right_foot" + _userNumber) {
		if (translation.x < _controlBox->front) {
			_rightXMotionControl = FORWARD;
		} else if (translation.x > _controlBox->back) {
			_rightXMotionControl = BACKWARD;
		} else {
			_rightXMotionControl = STOPX;
		}

		if (translation.y < _controlBox->left) {
			_rightYMotionControl = RIGHT;
		} else {
			_rightYMotionControl = STOPY;
		}
		constructAndPublishMovement();
	}

	if (child == "/torso" + _userNumber) {
		float x = transform.transform.rotation.x;
		float y = transform.transform.rotation.y;
		float z = transform.transform.rotation.z;
		float w = transform.transform.rotation.w;
		tf::Quaternion quat(x, y, z, w);
		tf::Matrix3x3 mat(quat);
		double roll, pitch, yaw;
		mat.getRPY(roll, pitch, yaw);

		_torsoRotation = pitch;
	}

}

void moveArms(geometry_msgs::TransformStamped& transform,
		geometry_msgs::Vector3& translation, bool toPublish)
{
	tf::Vector3 origin(translation.x, translation.y, translation.z);

	// Initialise roll, pitch and yaw values for joints
	float x = transform.transform.rotation.x;
	float y = transform.transform.rotation.y;
	float z = transform.transform.rotation.z;
	float w = transform.transform.rotation.w;
	tf::Quaternion quat(x, y, z, w);
	tf::Matrix3x3 mat(quat);
	double roll, pitch, yaw;
	mat.getRPY(roll, pitch, yaw);

	// Vectors for publishing to Nao (ex. joint at joints[i] moves to angles[i] angle)
	std::vector < std::string > joints;
	std::vector<float> angles;

	if (transform.child_frame_id.find("left_shoulder" + _userNumber) != std::string::npos) {
		_leftShoulder = new tf::Vector3(origin);
		pitch = -yaw;
		if (pitch > -2.0857 && pitch < 2.0857) {
			joints.push_back("LShoulderPitch");
			angles.push_back((float) (pitch));
			toPublish = true;
		}
		if (roll > -0.3142 && roll < 1.3265) {
			joints.push_back("LShoulderRoll");
			angles.push_back((float) (roll));
			toPublish = true;
		}
	}

	if (transform.child_frame_id.find("right_shoulder" + _userNumber)
			!= std::string::npos) {
		_rightShoulder = new tf::Vector3(origin);
		pitch = yaw;
		roll = -roll - (M_PI / 4);

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

	//	printf("Right shoulder roll: %f, pitch: %f      \r", roll, pitch);
	//	std::flush(std::cout);
	}

	if (transform.child_frame_id.find("left_elbow" + _userNumber) != std::string::npos) {
		_leftElbow = new tf::Vector3(origin);
		yaw = -pitch;
		roll = _leftElbowRoll;
		if (yaw > -2.0857 && yaw < 2.0857) {
			joints.push_back("LElbowYaw");
			angles.push_back((float) (yaw));
			toPublish = true;
		}
		if (roll > -1.5446 && roll < -0.0349) {
			joints.push_back("LElbowRoll");
			angles.push_back((float) (roll));
			toPublish = true;
		}
	}

	if (transform.child_frame_id.find("right_elbow" + _userNumber) != std::string::npos) {
		_rightElbow = new tf::Vector3(origin);
		yaw = pitch;
		roll = _rightElbowRoll;
		if (yaw > -2.0857 && yaw < 2.0857) {
			joints.push_back("RElbowYaw");
			angles.push_back((float) (pitch));
			toPublish = true;
		}
		if (roll > 0.0349 && roll < 1.5446) {
			joints.push_back("RElbowRoll");
			angles.push_back((float) (roll));
			toPublish = true;
		}
	}

	// Elbow roll must be calculated manually
	if (transform.child_frame_id.find("left_hand" + _userNumber) != std::string::npos) {
		_leftHand = new tf::Vector3(origin);
	}
	if (transform.child_frame_id.find("right_hand" + _userNumber) != std::string::npos) {
		_rightHand = new tf::Vector3(origin);
	}
	calculateLElbowRoll();
	calculateRElbowRoll();

	if (toPublish) {
		constructAndPublishMsg(joints, angles);
	}
}

#define MIN_DISTANCE 0.5

void lookForStartGesture(const geometry_msgs::TransformStamped& transform,
						const geometry_msgs::Vector3& translation)
{
	std::string child = transform.child_frame_id;
	std::string userNum = child.substr(child.size() - 2);

	if (child.find("head") != std::string::npos) {
		_userDistanceMap[userNum].first = translation;
	}
	if (child.find("left_hand") != std::string::npos) {
		_userDistanceMap[userNum].second = translation;
	}

	try {
		// Use torso frame for initialising control box
		if (child.find("torso") != std::string::npos) {
			geometry_msgs::Vector3 head = _userDistanceMap.at(userNum).first;
			geometry_msgs::Vector3 hand = _userDistanceMap.at(userNum).second;

			// Lazy distance calculation
			tf::Vector3 he(head.x, head.y, head.z);
			tf::Vector3 ha(hand.x, hand.y, hand.z);
			float dist = he.distance(ha);

			if (dist <= MIN_DISTANCE) {
				_userNumber = userNum;

				// Signal to TfRemap node which user to track
				std_msgs::String msg;
				msg.data = _userNumber;
				_user_pub.publish(msg);

				printf("User set: %s           \n", userNum.c_str());
				init(transform);
				ready = true;
			}
		}
	} catch (std::out_of_range& ex) { }
}

void tfCallback(const tf::tfMessage msg)
{
	bool toPublish = false;
	geometry_msgs::TransformStamped transform = msg.transforms[0];

	// Only allow application to continue when message containing tf for head joint is received.
	checkTfBroadcasting(transform);

	geometry_msgs::Vector3 translation = transform.transform.translation;

	if (broadcasting) {
		if (ready) {
			// Move arms
			if (transform.header.frame_id == REMAP_FRAME) {
				moveArms(transform, translation, toPublish);
			}
			// Control walking
			if (transform.header.frame_id == ORIGINAL_FRAME) {
				moveLegs(transform, translation);
			}
		} else {
			if (transform.header.frame_id == ORIGINAL_FRAME) {
				lookForStartGesture(transform, translation);
			}
		}
	}
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "mimicker");
	ros::NodeHandle nh;
	_arms_pub = nh.advertise<nao_msgs::JointAnglesWithSpeed>("/joint_angles", 50);
	_walk_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 2);
	_user_pub = nh.advertise<std_msgs::String>("/user_number", 1);
	ros::Subscriber sub = nh.subscribe("/tf", 50, tfCallback);

	// Short delay to allow for stepping back from laptop
	sleep(2);

	ros::Rate rate(60);
	ROS_INFO("Waiting for a head tf to be published.");
	while (!broadcasting) {
		ros::spinOnce();

		if (!nh.ok()) {
			ROS_INFO("Closing down.");

			exit(EXIT_FAILURE);
		}

		rate.sleep();
	}

	ROS_INFO("All set up, go!");
	ros::spin();

	// Send message to stop moving once the program has finished
	geometry_msgs::Twist stopMsg;
	stopMsg.linear.x = 0.0;
	stopMsg.linear.y = 0.0;
	_walk_pub.publish(stopMsg);

	return 0;
}



