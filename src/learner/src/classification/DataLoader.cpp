/*
 * dataLoader.cpp
 *
 *  Created on: 3 Jul 2013
 *      Author: tristan
 */

#include <classification/DataLoader.h>
#include <boost/foreach.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
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

vector<classification::DataPoint*> classification::DataLoader::loadData(
		string filename)
{
	vector<DataPoint*> poses;

	try {
		rosbag::Bag bag(filename);
		rosbag::View view(bag, rosbag::TopicQuery("/tf"));

		// Vector for generating PoseData (re-used when each is created)
		vector<geometry_msgs::TransformStamped> transforms;

		// Iterate over all messages published to this topic
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

						// Save this data point to file
						pose.serialize("test.thing");

						poses.push_back(new PoseDataPoint(pose));
						transforms.clear();
					}

					transforms.push_back(fullTransform);
				}
			}
		}

		bag.close();
//		out.close();
//		archive.close();
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

string classification::DataLoader::findFile(string directory,
		boost::posix_time::ptime timestamp)
{
	using namespace boost::filesystem;

//	cout << timestamp.date() << " " << timestamp.time_of_day() << endl << endl;

	path filepath(directory);
	if (!exists(filepath)) {
		// TODO: throw exception
		cout << "Directory " << directory << " does not exist" << endl;
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
		// Construct a string that matches the format for boost::posix_time::time_from_string argument
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
	}
	else {
		return paths[latestIndex - 1].generic_string();
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
//					cout << "Pushing back!" << endl;
					line1 += "\n";
					line2 += "\n";
					line3 += "\n";
					line1 += line2 += line3;
					timestamps.push_back(line1);

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
		// Deal with final line
		if (line1 != "" && line2 != "" && line3 != "") {
//			cout << "Pushing back!" << endl;
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

/*
 * Takes a single timestamp (3 lines: behavior, prompt, correct/incorrect)
 * and modifies start and end times accordingly.
 */
void classification::DataLoader::parseTimestamp(string timestamp,
		ros::Time &start, ros::Time &end, string &behaviorName)
{
	using namespace boost;

	vector<string> fields;

	// If you get an error here, it's just eclipse being a dick. Works if you compile from terminal.
	split(fields, timestamp, is_any_of("\n"));

	double startTime = atof((fields[1].substr(1, 20)).c_str()) - 0.5;
	double endTime = atof((fields[2].substr(1, 20)).c_str());

//	ros::Time::init();
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
		if (behavior[i].size() > 0) {
			behaviorName = behavior[i];
			break;
		}
	}
}

vector<classification::DataPoint*> classification::DataLoader::getDataSubset(
		vector<DataPoint*> &data, ros::Time start, ros::Time end)
{
	vector<DataPoint*> subset;

	double startTime = floor(start.toSec());
	double endTime = floor(end.toSec());

	// Find and push all DataPoints between start and end times
	BOOST_FOREACH(DataPoint *point, data){
	double time = floor(point->getTimestamp().toSec());

	if (time >= startTime && time <= endTime) {
		subset.push_back(point);
	}
	else if (time > endTime) {
		break;
	}
}

	return subset;
}

void classification::DataLoader::filterData(string filename)
{
	vector<string> ts = classification::DataLoader::readTimestampFile(filename);

	rosbag::Bag happyBag("happy.bag", rosbag::bagmode::Write);
	rosbag::Bag sadBag("sad.bag", rosbag::bagmode::Write);
	rosbag::Bag angryBag("angry.bag", rosbag::bagmode::Write);
	rosbag::Bag scaredBag("scared.bag", rosbag::bagmode::Write);

	ros::Time::init();
	ros::Time timeToWrite = ros::Time::now();
	ros::Duration timeToAdd(PLAYBACK_SPEED);

	for (int i = 0; i < ts.size(); i++) {
		printf("Adding timestamp %d...\n", i + 1);

		string timestamp = ts[i];

		// Get the start and end times for the timestamp
		ros::Time start;
		ros::Time end;
		string behavior;

		cout << "   Parsing timestamp..." << endl;
		parseTimestamp(timestamp, start, end, behavior);

		// Find the file that contains the timestamp
		cout << "   Finding file..." << endl;
		string filepath = findFile("/home/tristan/nao-autism/recordings/",
				start.toBoost() + boost::posix_time::hours(1));
		cout << "   " << filepath << endl;

		// Construct training data from that file
		classification::TrainingData poses = loadData(filepath);

		// Get the subset of data that corresponds to the timestamp
		classification::TrainingData subset = getDataSubset(poses, start, end);

		if (behavior == "happy")
			writeToFile(happyBag, subset, behavior, timeToWrite);
		else if (behavior == "sad")
			writeToFile(sadBag, subset, behavior, timeToWrite);
		else if (behavior == "angry")
			writeToFile(angryBag, subset, behavior, timeToWrite);
		else if (behavior == "scared")
			writeToFile(scaredBag, subset, behavior, timeToWrite);
		else
			cerr << "Unknown behavior name found" << endl;

		timeToWrite += timeToAdd;
		printf("...Done\n\n");
	}

	happyBag.close();
	sadBag.close();
	angryBag.close();
	scaredBag.close();
}

void classification::DataLoader::writeToFile(rosbag::Bag &bag,
		classification::TrainingData data, std::string behaviorName,
		ros::Time &timeToWrite)
{
	vector<classification::PoseDataPoint*> dataPoints =
			PoseDataPoint::convertToPoses(data);

	BOOST_FOREACH(PoseDataPoint* pose, dataPoints){
		(pose->poseData).writeToFile(bag, timeToWrite);
	}
}

//int main(int argc, char **argv)
//{
//	ros::init(argc, argv, "data_loader");
//
//	ROS_INFO("Starting...\n");
//
//	classification::DataLoader::filterData("timestamps.log");
//
//	ROS_INFO("Finished!");
//
//	return 0;
//}

