/*
 * TfRemap.cpp
 *
 *  Created on: Aug 7, 2013
 *      Author: parallels
 */

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <boost/foreach.hpp>

#include <vector>
#include <string>

#define FRAME_ID "tf_remap"

class TfRemap {

public:
	tf::TransformBroadcaster _broadcaster;
	const std::string _reference_frame;
	std::vector<std::string> _joints;

	TfRemap(void) : _broadcaster(), _reference_frame("/torso_1") {
		ros::Time::init();

		_joints.push_back("/head_1");
		_joints.push_back("/neck_1");
		_joints.push_back("/left_shoulder_1");
		_joints.push_back("/left_elbow_1");
		_joints.push_back("/left_hand_1");
		_joints.push_back("/right_shoulder_1");
		_joints.push_back("/right_elbow_1");
		_joints.push_back("/right_hand_1");
		_joints.push_back("/left_hip_1");
		_joints.push_back("/left_knee_1");
		_joints.push_back("/left_foot_1");
		_joints.push_back("/right_hip_1");
		_joints.push_back("/right_knee_1");
		_joints.push_back("/right_foot_1");
	}

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

int main(int argc, char **argv) {
	ros::init(argc, argv, "remapper");
	TfRemap remapper;
	remapper.remap();

	return 0;
}


























