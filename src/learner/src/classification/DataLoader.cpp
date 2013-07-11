/*
 * dataLoader.cpp
 *
 *  Created on: 3 Jul 2013
 *      Author: tristan
 */

#include <DataLoader.h>
#include <boost/foreach.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <geometry_msgs/TransformStamped.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <tf/tfMessage.h>

#include <iostream>
#include <stdlib.h>
#include <math.h>

using namespace std;

vector<classification::DataPoint*> classification::DataLoader::loadData(string filename)
{
	vector<DataPoint*> poses;

	try {
		rosbag::Bag bag(filename);
		rosbag::View view(bag, rosbag::TopicQuery("/tf"));

		// Vector for generating PoseData (re-used when each is created)
		vector<geometry_msgs::TransformStamped> transforms;

		// Iterate over all messages published to this topic
		BOOST_FOREACH(rosbag::MessageInstance const m, view) {
			tf::tfMessage::ConstPtr i = m.instantiate<tf::tfMessage>();

			if (i != NULL) {
				geometry_msgs::TransformStamped fullTransform = (i->transforms)[0];

				// Only deal with transforms from joints (not camera frame transforms)
				if (fullTransform.child_frame_id.find("camera") == string::npos &&
						fullTransform.header.frame_id == "/openni_depth_frame") {
					// If 'transforms' is full of joint data, create a PoseDataPoint object
					// from it, push it onto 'poses', then reset it.
					if (transforms.size() == 15) {
						PoseData *p = extractPose(transforms);
						PoseDataPoint pose(*p);
						poses.push_back(new PoseDataPoint(pose));
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

PoseData *classification::DataLoader::extractPose(
		const vector<geometry_msgs::TransformStamped> transforms)
{
	PoseData *pose = new PoseData;

	// Can either assume 'transforms' has joints in right order or check, I am assuming
	// they are in order for speed.
	pose->head = transforms[0];
	pose->neck = transforms[1];
	pose->torso = transforms[2];
	pose->left_shoulder = transforms[3];
	pose->left_elbow = transforms[4];
	pose->left_hand = transforms[5];
	pose->right_shoulder = transforms[6];
	pose->right_elbow = transforms[7];
	pose->right_hand = transforms[8];
	pose->left_hip = transforms[9];
	pose->left_knee = transforms[10];
	pose->left_foot = transforms[11];
	pose->right_hip = transforms[12];
	pose->right_knee = transforms[13];
	pose->right_foot = transforms[14];

	return pose;
}

string classification::DataLoader::findFile(string directory, boost::posix_time::ptime timestamp)
{
	using namespace boost::filesystem;

//	cout << timestamp.date() << " " << timestamp.time_of_day() << endl << endl;

	path filepath(directory);
	if (!exists(filepath)) {
		// TODO: throw exception
	}

	vector<path> paths;
	copy(directory_iterator(filepath), directory_iterator(),
			back_inserter(paths));
	sort(paths.begin(), paths.end());

	vector<boost::posix_time::ptime> times;
	/*for (vector<path>::const_iterator it(paths.begin()); it != paths.end(); ++it) {
		cout << "   " << *it << '\n';
	}*/

	int latestIndex = 0;

	for (int i = 0; i < paths.size(); i++) {
		if (i == paths.size() - 1) break;

		string fileTime = paths[i].filename().generic_string();
		fileTime = fileTime.substr(1, 19);
		fileTime[10] = ' ';
		fileTime[13] = ':';
		fileTime[16] = ':';

//		cout << fileTime << endl;
		boost::posix_time::ptime fTime(boost::posix_time::time_from_string(fileTime));
//		cout << (timestamp < fTime ? "Too late" : "Too early") << endl;

		if (timestamp < fTime) {
			latestIndex = i;
			break;
		}
	}

	if (latestIndex == 0) {
		// Earliest recording is later than the timestamp, so not found.
		// Should probably throw an exception instead.
		return "";
	} else {
		return paths[latestIndex-1].generic_string();
	}


}

/*
 * Takes a single timestamp (3 lines: behavior, prompt, correct/incorrect)
 * and modifies start and end times accordingly.
 */
void classification::DataLoader::parseTimestamp(string timestamp, ros::Time &start, ros::Time &end, string &behaviorName)
{
	using namespace boost;

	vector <string> fields;

	// If you get an error here, it's just eclipse being a dick. Works if you compile from terminal.
	split(fields, timestamp, is_any_of("\n"));

//	for (int i = 0; i < fields.size(); i++) {
//		cout << "Line " << i << ": " << fields[i] << endl;
//	}

	double startTime = atof((fields[1].substr(1, 20)).c_str()) - 1.5;
	double endTime = atof((fields[2].substr(1, 20)).c_str());

	ros::Time::init();
	ros::Time s(startTime);
	ros::Time e(endTime);

	start = s;
	end = e;

	vector <string> behavior;
	// Filter out everything but the behavior name (hacky, i know)
	split(behavior, fields[0], is_any_of("ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_=[]. "), token_compress_on);

	for (int i = 0; i < behavior.size(); i++) {
		if (behavior[i].size() > 0) {
			behaviorName = behavior[i];
			break;
		}
	}
}

vector<classification::DataPoint*> classification::DataLoader::getDataSubset(vector<DataPoint*> &data, ros::Time start, ros::Time end)
{
	vector<DataPoint*> subset;

	double startTime = floor(start.toSec());
	double endTime = floor(end.toSec());

	// Find and push all DataPoints between start and end times
	BOOST_FOREACH(DataPoint *point, data) {
		double time = floor(point->getTimestamp().toSec());

		if (time >= startTime && time <= endTime) {
			subset.push_back(point);
		} else if (time > endTime) {
			break;
		}
	}

	return subset;
}

/*int main(int argc, char **argv) {
	ros::init(argc, argv, "data_loader");

	ROS_INFO("Starting...");

	vector<DataPoint*> poses = DataLoader::loadData(
			"/home/tristan/nao-autism/recordings/happy_2013-07-03-09-58-54.bag");

	for (int i = 0; i < poses.size(); i++) {
		cout << "	POSE " << i << ":" << endl;
		cout << poses[i].head << endl;
		cout << poses[i].neck << endl;
		cout << poses[i].torso << endl;
		cout << poses[i].left_shoulder << endl;
		cout << poses[i].left_elbow << endl;
		cout << poses[i].left_hand << endl;
		cout << poses[i].right_shoulder << endl;
		cout << poses[i].right_elbow << endl;
		cout << poses[i].right_hand << endl;
		cout << poses[i].left_hip << endl;
		cout << poses[i].left_knee << endl;
		cout << poses[i].left_foot << endl;
		cout << poses[i].right_hip << endl;
		cout << poses[i].right_knee << endl;
		cout << poses[i].right_foot << endl;
	}

	ROS_INFO("Finished!");

	return 0;
}*/





















