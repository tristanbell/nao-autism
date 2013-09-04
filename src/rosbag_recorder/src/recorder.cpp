/*
 * Recorder.cpp
 *
 *  Created on: 2 Jul 2013
 *      Author: Tristan
 */

#include <recorder.h>
#include <boost/thread.hpp>

const float Recorder::RECORDING_DURATION = 25.0; // Currently unused
const uint32_t Recorder::RECORDING_SIZE = 1073741824; // Record a gigabyte before splitting
int recorderResult; // Currently unused
bool currentlyRecording;

Recorder::Recorder(void)
{
	// Set topics to subscribe to
	std::vector<std::string> topicsToRecord;
	topicsToRecord.push_back("/tf");
//	topicsToRecord.push_back("camera/rgb/image_raw");
//	topicsToRecord.push_back("camera/rgb/camera_info");
//	topicsToRecord.push_back("camera/depth_registered/image_raw");
//	topicsToRecord.push_back("camera/depth_registered/camera_info");
	options.topics = topicsToRecord;

	options.prefix = "recordings/";

	/*// Set duration of recording
	ros::Duration dur(RECORDING_DURATION);
	options.max_duration = dur;*/

	options.max_size = RECORDING_SIZE;
	options.split = true;
}

void stop(void)
{
	// Unimplemented
}

void Recorder::record(void)
{
	Recorder rec;
	rosbag::Recorder recorder(rec.options);
	recorderResult = recorder.run();
}


void Recorder::recordCallback(const nao_autism_messages::Record::ConstPtr& msg)
{
	if (msg->record && !currentlyRecording) {
		ROS_INFO("Recording");
		currentlyRecording = true;
		Recorder::record();
	}
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "recorder");

	ros::NodeHandle node;

	ros::Subscriber sub = node.subscribe("record", 10, Recorder::recordCallback);
	ros::spin();

	return 0;
}


