/*
 * TfRemap.cpp
 *
 *  Created on: Aug 7, 2013
 *      Author: parallels
 */

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/tfMessage.h>
#include <std_msgs/String.h>
#include <boost/foreach.hpp>

#include <vector>
#include <string>

#define FRAME_ID "tf_remap"

class TfRemap {

private:
	bool ready;

	void waitForTfBroadcast(tf::tfMessage msg) {
		if (msg.transforms[0].child_frame_id.find("head")
						!= std::string::npos) {

			ROS_INFO("/tf is now publishing, ready to go.");
			ready = true;
		}
	}

public:
	tf::TransformBroadcaster _broadcaster;
	const std::string _reference_frame;
	std::vector<std::string> _joints;

	TfRemap(std::string userNum) : _broadcaster(), _reference_frame("/torso" + userNum) {
		ros::Time::init();

		_joints.push_back("/head" + userNum);
		_joints.push_back("/neck" + userNum);
		_joints.push_back("/left_shoulder" + userNum);
		_joints.push_back("/left_elbow" + userNum);
		_joints.push_back("/left_hand" + userNum);
		_joints.push_back("/right_shoulder" + userNum);
		_joints.push_back("/right_elbow" + userNum);
		_joints.push_back("/right_hand" + userNum);
		_joints.push_back("/left_hip" + userNum);
		_joints.push_back("/left_knee" + userNum);
		_joints.push_back("/left_foot" + userNum);
		_joints.push_back("/right_hip" + userNum);
		_joints.push_back("/right_knee" + userNum);
		_joints.push_back("/right_foot" + userNum);

		ros::NodeHandle nh;
		ros::Subscriber tfSub = nh.subscribe("/tf", 10, &TfRemap::waitForTfBroadcast, this);
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
	}

	/**
	 * Remap transforms from the original frame to their position relative
	 * to the torso.
	 */
	void remap(void) {
		ros::NodeHandle _node;
		tf::TransformListener _listener;

		ros::Rate _rate(200.0);

		while (_node.ok()) {
			std::vector<tf::StampedTransform> transforms(_joints.size());

			try {
				for (int i = 0; i < _joints.size(); i++) {
					_listener.waitForTransform(_reference_frame,_joints[i],
							ros::Time(0), ros::Duration(5.0));
					_listener.lookupTransform(_reference_frame, _joints[i],
							ros::Time(0), transforms[i]);

					transforms[i].child_frame_id_ = _joints[i];
				}
			} catch (tf::TransformException& ex) {
				ROS_ERROR("%s", ex.what());
			}


			BOOST_FOREACH(tf::StampedTransform transform, transforms) {
				if (transform.child_frame_id_ != "") {
					transform.frame_id_ = FRAME_ID;
					transform.stamp_ = ros::Time::now();

					// Re-broadcast
					_broadcaster.sendTransform(transform);
				}
			}

			transforms.clear();
			_rate.sleep();
		}
	}
};

std::string userNumber = "";

/**
 * Sets the user number to track.
 */
void userNumCallback(std_msgs::String msg)
{
	userNumber = msg.data;
	std::cout << "User set for remapping." << std::endl;
}

/**
 * Waits for a user number to be sent that determines which user to track.
 */
void waitForUserNumber(void)
{
	ros::NodeHandle nh;
	ros::Subscriber sub = nh.subscribe("/user_number", 1, userNumCallback);

	ros::Rate r(30);
	while (userNumber == "") {
		ros::spinOnce();
		r.sleep();
	}


}

int main(int argc, char **argv) {
	ros::init(argc, argv, "remapper");

	waitForUserNumber();
	TfRemap remapper(userNumber);
	remapper.remap();

	return 0;
}


























