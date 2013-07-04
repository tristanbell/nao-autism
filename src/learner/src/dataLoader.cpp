/*
 * dataLoader.cpp
 *
 *  Created on: 3 Jul 2013
 *      Author: tristan
 */

#include <dataLoader.h>
#include <boost/foreach.hpp>
#include <geometry_msgs/TransformStamped.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <tf/tfMessage.h>

#include <iostream>

std::vector<PoseData> DataLoader::loadData(std::string filename)
{
	std::vector<PoseData> poses;

	try {
		rosbag::Bag bag(filename);
		rosbag::View view(bag, rosbag::TopicQuery("/tf"));

		// Vector for generating PoseData (re-used when each is created)
		std::vector<geometry_msgs::TransformStamped> transforms;

		// Iterate over all messages published to this topic
		BOOST_FOREACH(rosbag::MessageInstance const m, view) {
			tf::tfMessage::ConstPtr i = m.instantiate<tf::tfMessage>();

			if (i != NULL) {
				geometry_msgs::TransformStamped fullTransform = (i->transforms)[0];

				// Only deal with transforms from joints (not camera frame transforms)
				if (fullTransform.child_frame_id.find("camera") == std::string::npos) {
					// If 'transforms' is full of joint data, create a PoseData object
					// from it, push it onto 'poses', then reset it.
					if (transforms.size() == 15) {
						PoseData pose = extractPose(transforms);
						poses.push_back(pose);
						transforms.clear();
					}

					transforms.push_back(fullTransform);
				}
			}
		}

		bag.close();

	} catch (rosbag::BagIOException &e) {
		ROS_ERROR("%s", e.what());
	}

	return poses;
}

PoseData DataLoader::extractPose(
		const std::vector<geometry_msgs::TransformStamped> transforms)
{
	PoseData pose;

	// Can either assume 'transforms' has joints in right order or check, I am assuming
	// they are in order for speed.
	pose.head = transforms[0];
	pose.neck = transforms[1];
	pose.torso = transforms[2];
	pose.left_shoulder = transforms[3];
	pose.left_elbow = transforms[4];
	pose.left_hand = transforms[5];
	pose.right_shoulder = transforms[6];
	pose.right_elbow = transforms[7];
	pose.right_hand = transforms[8];
	pose.left_hip = transforms[9];
	pose.left_knee = transforms[10];
	pose.left_foot = transforms[11];
	pose.right_hip = transforms[12];
	pose.right_knee = transforms[13];
	pose.right_foot = transforms[14];

	return pose;
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "data_loader");

	ROS_INFO("Starting...");

	std::vector<PoseData> poses = DataLoader::loadData(
			"/home/tristan/nao-autism/recordings/happy_2013-07-03-09-58-54.bag");

	/*for (int i = 0; i < poses.size(); i++) {
		std::cout << "	POSE " << i << ":" << std::endl;
		std::cout << poses[i].head << std::endl;
		std::cout << poses[i].neck << std::endl;
		std::cout << poses[i].torso << std::endl;
		std::cout << poses[i].left_shoulder << std::endl;
		std::cout << poses[i].left_elbow << std::endl;
		std::cout << poses[i].left_hand << std::endl;
		std::cout << poses[i].right_shoulder << std::endl;
		std::cout << poses[i].right_elbow << std::endl;
		std::cout << poses[i].right_hand << std::endl;
		std::cout << poses[i].left_hip << std::endl;
		std::cout << poses[i].left_knee << std::endl;
		std::cout << poses[i].left_foot << std::endl;
		std::cout << poses[i].right_hip << std::endl;
		std::cout << poses[i].right_knee << std::endl;
		std::cout << poses[i].right_foot << std::endl;
	}*/

	ROS_INFO("Finished!");

	return 0;
}





















