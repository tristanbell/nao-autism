/*
 * TFMeasure.cpp
 *
 *  Created on: Aug 16, 2013
 *      Author: parallels
 */

#include <ros/ros.h>
#include <tf/tfMessage.h>
#include <geometry_msgs/TransformStamped.h>
#include <vector>
#include <string>

using namespace std;

vector<string> joints;

tf::tfMessage* joint1;
tf::tfMessage* joint2;

void tfCallback(const tf::tfMessage msg) {
	geometry_msgs::TransformStamped transform = msg.transforms[0];
	if (joints.size() == 1) {
		printf("x: %f, y: %f, z: %f\r", transform.transform.translation.x,
				transform.transform.translation.y,
				transform.transform.translation.z);
		flush(cout);
	} else if (joints.size() == 2) {
		if (transform.child_frame_id == "/" + joints[0])
			joint1 = new tf::tfMessage(msg);
		if (transform.child_frame_id == "/" + joints[1])
			joint2 = new tf::tfMessage(msg);

		if (joint1 && joint2) {
			float xDist = joint1->transforms[0].transform.translation.x
					- joint2->transforms[0].transform.translation.x;
			float yDist = joint1->transforms[0].transform.translation.y
					- joint2->transforms[0].transform.translation.y;
			float zDist = joint1->transforms[0].transform.translation.z
					- joint2->transforms[0].transform.translation.z;

			float dist = sqrt(xDist * xDist + yDist * yDist + zDist * zDist);

			cout << dist << "\r";
			flush(cout);
		}
	}
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "measurer");

	if (argc < 2 || argc > 4) {
		ROS_ERROR("Please supply one or two joints to track.");
		return -1;
	}

	for (int i = 1; i < argc; i++) {
		joints.push_back(string(argv[i]));
	}

	ros::NodeHandle nh;
	ros::Subscriber sub = nh.subscribe("/tf", 15, tfCallback);

	ros::spin();

	cout << endl;

	return 0;
}

