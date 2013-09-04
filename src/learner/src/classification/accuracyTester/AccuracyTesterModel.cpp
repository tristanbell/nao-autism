/*
 * AccuracyTester.cpp
 *
 *  Created on: Sep 3, 2013
 *      Author: parallels
 */

#include <classification/accuracyTester/AccuracyTesterModel.h>

classification::AccuracyTesterModel::AccuracyTesterModel() : _nh()
{
	_sub = _nh.subscribe("/classification", 10,
			&classification::AccuracyTesterModel::classificationCallback, this);
}

void classification::AccuracyTesterModel::classificationCallback(
		nao_autism_messages::PoseClassification msg)
{
	_accuracy_map[msg.source][msg.classification].push_back(msg.classification == _expected_class);
}

float classification::AccuracyTesterModel::getAccuracy(string learning_name, int classification)
{
	float numRight;
	vector<bool> accuracyVec = _accuracy_map[learning_name][classification];

	for (int i = 0; i < accuracyVec.size(); i++) {
		if (accuracyVec[i])
			numRight++;
	}

	return numRight / accuracyVec.size();
}
