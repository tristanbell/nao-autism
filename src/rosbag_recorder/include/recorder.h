/*
 * recorder2.h
 *
 *  Created on: 2 Jul 2013
 *      Author: tristan
 */

#ifndef RECORDER_H_
#define RECORDER_H_

#include <rosbag/recorder.h>
#include <ros/ros.h>
#include <vector>
#include <string>

class Recorder {
public:
	static void record(std::string emotionName);
	void stop(void);
	static const float RECORDING_DURATION;
	static const uint32_t RECORDING_SIZE;

private:
	Recorder(std::string emotion);

	rosbag::RecorderOptions options;
};

#endif /* RECORDER_H_ */
