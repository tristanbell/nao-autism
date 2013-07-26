/*
 * SVMListener.cpp
 *
 *  Created on: Jul 26, 2013
 *      Author: parallels
 */

#include <ros/ros.h>
#include <learner/PoseClassification.h>

#include <vector>
#include <map>
#include <exception>

std::vector<learner::PoseClassification> _poseQueue;
short _classification;

#define MAX_QUEUE_SIZE 15

void setOverallClass(void) {
	std::map<short, int> votes;

	// Get the number of times each pose appears in _poseQueue
	for (int i = 0; i < _poseQueue.size(); i++) {
		votes[_poseQueue[i].classification]++;
	}

	// Get the pose with the highest number of votes
	std::map<short, int>::iterator it = votes.begin();
	std::pair<short, int> pair = *it;

	short highestPose = pair.first;
	int highestNum = pair.second;

	it++;

	while (it != votes.end()) {
		pair = *it;

		if (pair.second > highestNum) {
			highestPose = pair.first;
			highestNum = pair.second;
		}

		it++;
	}

	_classification = highestPose;

	_poseQueue.clear();
}

void classificationCallback(const learner::PoseClassification poseClass) {
	if (_poseQueue.size() >= MAX_QUEUE_SIZE) {
		setOverallClass();
		printf("Current class: %d         \n", _classification);
	}

	_poseQueue.push_back(poseClass);
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "class_listener");

	ros::NodeHandle nh;
	ros::Subscriber sub = nh.subscribe("/classification", 50, classificationCallback);

	ros::spin();

	return 0;
}

