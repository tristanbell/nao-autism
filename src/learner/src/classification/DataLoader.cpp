/*
 * dataLoader.cpp
 *
 *  Created on: 3 Jul 2013
 *      Author: tristan
 */

#include <classification/DataLoader.h>
#include <boost/foreach.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <geometry_msgs/TransformStamped.h>
#include <rosbag/view.h>
#include <tf/tfMessage.h>

#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <math.h>

using namespace std;

const float PLAYBACK_SPEED = 0.043;
classification::TrainingData _standingData;

vector<classification::DataPoint*> classification::DataLoader::loadData(
		string filename)
{
	vector<DataPoint*> poses;

	try {
		rosbag::Bag bag(filename);
		rosbag::View view(bag, rosbag::TopicQuery("/tf"));

		// Vector for generating PoseData (re-used when each is created)
		vector<geometry_msgs::TransformStamped> transforms;

		// Iterate over all messages published to this topic and construct complete poses
		BOOST_FOREACH(rosbag::MessageInstance const m, view){
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

int classification::DataLoader::getFileIndex(vector<boost::filesystem::path>& paths,
		boost::posix_time::ptime timestamp)
{
	int latestIndex = 0;

	for (int i = 0; i < paths.size(); i++) {
		// The name of the recorded bagfile is the date it was recorded
		string fileTime = paths[i].filename().generic_string();

		// Take this name and modify it to the format accepted by
		// boost::posix_time::time_from_string()
		fileTime = fileTime.substr(1, 19);
		fileTime[10] = ' ';
		fileTime[13] = ':';
		fileTime[16] = ':';
		boost::posix_time::ptime fTime(
				boost::posix_time::time_from_string(fileTime));

		// Because 'paths' is assumed to be sorted, the last path that was created
		// before the timestamp was must be the file we need.
		if (timestamp < fTime) {
			latestIndex = i-1;
			break;
		}
	}

	return latestIndex;
}

string classification::DataLoader::findFile(string directory,
		boost::posix_time::ptime timestamp)
{
	using namespace boost::filesystem;

	path filepath(directory);
	if (!exists(filepath)) {
		// TODO: exception
		cout << "Directory " << directory << " does not exist" << endl;
	}

	// Get all the files contained within the directory and sort by name.
	// Assuming the files are named with the dates they were recorded,
	// this will put them in date order.
	vector < path > paths;
	copy(directory_iterator(filepath), directory_iterator(),
			back_inserter(paths));
	sort(paths.begin(), paths.end());
	vector < boost::posix_time::ptime > times;

	int fileIndex = getFileIndex(paths, timestamp);

	if (fileIndex == -1) {
		return "";
	}
	else {
		return paths[fileIndex].generic_string();
	}

}

vector<string> classification::DataLoader::readTimestampFile(string filename)
{
	vector<string> timestamps;

	ifstream in;
	in.open(filename.c_str());
	if (in.is_open()) {
		string tmp, line1, line2, line3;
		while (in.good()) {
			getline(in, tmp);

			if (tmp.find("BEHAVIOR_BUTTON") != string::npos) {
				// If all lines filled, append together and push to timestamps
				if (line1 != "" && line2 != "" && line3 != "") {
					line1 += "\n";
					line2 += "\n";
					line3 += "\n";
					line1 += line2 += line3;
					timestamps.push_back(line1);

					// Then reset line variables
					line1 = line2 = line3 = "";
				}
				line1 = tmp;
			}
			else if (tmp.find("PROMPT_BUTTON") != string::npos) {
				line2 = tmp;
			}
			else if (tmp.find("CORRECT_BUTTON") != string::npos) {
				line3 = tmp;
			}
			//TODO: deal with INCORRECT_BUTTON
		}
		// Deal with final line (off by one)
		if (line1 != "" && line2 != "" && line3 != "") {
			line1 += "\n";
			line2 += "\n";
			line3 += "\n";
			line1 += line2 += line3;
			timestamps.push_back(line1);

			line1 = line2 = line3 = "";
		}

		in.close();
	}
	else {
		// TODO: exception?
		cout << "Could not open file " << filename << endl;
	}

	return timestamps;
}

void classification::DataLoader::parseTimestamp(string timestamp,
		ros::Time &start, ros::Time &end, string &behaviorName)
{
	using namespace boost;

	vector<string> fields;

	// If you get an error here, it's just eclipse being a dick. Works if you compile from terminal.
	split(fields, timestamp, is_any_of("\n"));

	// The first part of the timestamp should be the time a button was clicked
	double startTime = atof((fields[1].substr(1, 20)).c_str()) + 0.5;
	double endTime = atof((fields[2].substr(1, 20)).c_str()) - 0.5;

	// Construct ros::Time variables for start and end times, then modify
	// the arguments accordingly.
	ros::Time s(startTime);
	ros::Time e(endTime);
	start = s;
	end = e;

	vector<string> behavior;
	// Filter out everything but the behavior name (hacky, i know)
	split(behavior, fields[0],
			is_any_of("ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_=[]. "),
			token_compress_on);

	for (int i = 0; i < behavior.size(); i++) {
		// Get the line with the behavior name in it
		if (behavior[i].size() > 0) {
			// Set the argument to the behavior name
			behaviorName = behavior[i];
			break;
		}
	}
}

vector<classification::DataPoint*> classification::DataLoader::getDataSubset(
		vector<DataPoint*> &data, ros::Time start, ros::Time end)
{
	vector<DataPoint*> subset;

	// Times are not totally accurate, floor them to round to closest second
	double startTime = floor(start.toSec());
	double endTime = floor(end.toSec());

	// Find and push all DataPoints between start and end times
	BOOST_FOREACH(DataPoint *point, data){
		double time = floor(point->getTimestamp().toSec());

		// Add 0.3 seconds to compensate for inaccuracies
		if (time >= startTime && time + 0.3 <= endTime) {
			subset.push_back(point);
		}
		else if (time + 0.3 > endTime) {
			break;
		}
	}

	return subset;
}

void classification::DataLoader::filterData(string filename)
{
	_standingData = classification::DataLoader::loadData("standing.bag");

	vector<string> ts = classification::DataLoader::readTimestampFile(filename);

	// Create bagfiles for each emotion
	rosbag::Bag happyBag("happy.bag", rosbag::bagmode::Write);
	rosbag::Bag sadBag("sad.bag", rosbag::bagmode::Write);
	rosbag::Bag angryBag("angry.bag", rosbag::bagmode::Write);
	rosbag::Bag scaredBag("scared.bag", rosbag::bagmode::Write);

	ros::Time::init();
	ros::Time happyTimeToWrite = ros::Time::now();
	ros::Time sadTimeToWrite = ros::Time::now();
	ros::Time scaredTimeToWrite = ros::Time::now();
	ros::Time angryTimeToWrite = ros::Time::now();

	for (int i = 0; i < ts.size(); i++) {
		printf("Adding timestamp %d...\n", i + 1);

		string timestamp = ts[i];

		// Get the start and end times for the timestamp, as well as the behavior name
		ros::Time start;
		ros::Time end;
		string behavior;
		cout << "   Parsing timestamp..." << endl;
		parseTimestamp(timestamp, start, end, behavior);

		// Find the file that contains the timestamp
		cout << "   Finding file..." << endl;
//		string filepath = findFile("/home/tristan/nao-autism/recordings/",
//				start.toBoost() + boost::posix_time::hours(1));
		string filepath = findFile("/media/Partition 1/recordings/",
				start.toBoost() + boost::posix_time::hours(1));
		cout << "   " << filepath << endl;

		// Construct training data from that file
		classification::TrainingData poses = loadData(filepath);

		// Get the subset of data that corresponds to the timestamp
		classification::TrainingData subset = getDataSubset(poses, start, end);

		// Write to the proper file
		if (behavior == "happy")
			writeToFile(happyBag, subset, happyTimeToWrite);
		else if (behavior == "sad")
			writeToFile(sadBag, subset, sadTimeToWrite);
		else if (behavior == "angry")
			writeToFile(angryBag, subset, angryTimeToWrite);
		else if (behavior == "scared")
			writeToFile(scaredBag, subset, scaredTimeToWrite);
		else
			cerr << "Unknown behavior name found" << endl;

		printf("...Done\n\n");
	}

	happyBag.close();
	sadBag.close();
	angryBag.close();
	scaredBag.close();
}

void classification::DataLoader::writeToFile(rosbag::Bag &bag,
		classification::TrainingData data,
		ros::Time &timeToWrite)
{
	ros::Duration timeToAdd(PLAYBACK_SPEED);
	vector<classification::PoseDataPoint*> dataPoints =
			PoseDataPoint::convertToPoses(data);

	BOOST_FOREACH(PoseDataPoint* pose, dataPoints){
		int rnd = rand() % (_standingData.size() - 1);
		DataPoint* pointToIgnore = _standingData[rnd];
//		printf("Choosing point %d of %d:\n", rnd, ((int)_standingData.size()));
//		printf("  Distance between points: %f        \n", pose->getDistance(*pointToIgnore));

		// Only write this data point to file if it is far enough away from pointToIgnore
		if (pose->getDistance(*pointToIgnore) > 10.0) {
			pose->writeToFile(bag, timeToWrite);
			// Simulates very short pauses between data (for playback in rviz)
			timeToWrite += timeToAdd;
		}
	}
}














