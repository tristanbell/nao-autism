/*
 * AccuracyTester.cpp
 *
 *  Created on: Sep 3, 2013
 *      Author: parallels
 */

#include <classification/accuracyTester/AccuracyTesterModel.h>

classification::AccuracyTesterModel::AccuracyTesterModel() :
		_nh() {
	_sub = _nh.subscribe("/classification", 50,
			&classification::AccuracyTesterModel::classificationCallback, this);
}

void classification::AccuracyTesterModel::classificationCallback(
		nao_autism_messages::PoseClassification msg) {
	_accuracy_map[msg.source][_expected_class].push_back(
			msg.classification == _expected_class);
}

float classification::AccuracyTesterModel::getAccuracy(string learning_name,
		int classification) {

	if (_accuracy_map.size() > 0) {
		float numRight;

		try {
			vector<bool> accuracyVec =
					_accuracy_map.at(learning_name).at(classification);

			for (int i = 0; i < accuracyVec.size(); i++) {
				if (accuracyVec[i])
					numRight++;
			}

			return numRight / accuracyVec.size() * 100.0;
		} catch (exception& e) {
			return 0.0;
		}
	}

	return 0.0;
}

void classification::AccuracyTesterModel::resetAccuracy() {
//	for (map<string, map<int, vector<bool> > >::iterator it =
//			_accuracy_map.begin(); it != _accuracy_map.end(); it++) {
//		pair<string, map<int, vector<bool> > > curr = *it;
//
//	}

	_accuracy_map.clear();
}
