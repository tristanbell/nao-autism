/*
 * Recorder.cpp
 *
 *  Created on: 2 Jul 2013
 *      Author: Tristan
 */

#include <recorder.h>

const float Recorder::RECORDING_DURATION = 25.0;
int recorderResult;

Recorder::Recorder(std::string emotionName)
{
	// Set topics to subscribe to
	std::vector<std::string> topicsToRecord(5);
	topicsToRecord.push_back("/tf");
	topicsToRecord.push_back("camera/rgb/image_raw");
	topicsToRecord.push_back("camera/rgb/camera_info");
	topicsToRecord.push_back("camera/depth_registered/image_raw");
	topicsToRecord.push_back("camera/depth_registered/camera_info");
	options.topics = topicsToRecord;

	options.prefix = "recordings/" + emotionName;

	// Set duration of recording
	ros::Duration dur(RECORDING_DURATION);
	options.max_duration = dur;
}

void stop(void)
{
	// Unimplemented
}

void Recorder::record(std::string emotionName)
{
	Recorder rec(emotionName);

	rosbag::Recorder recorder(rec.options);
	recorderResult = recorder.run();
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "recorder");

	Recorder::record("happy");

	return recorderResult;
}


