/*
 * AccuracyTester.h
 *
 *  Created on: Sep 3, 2013
 *      Author: parallels
 */

#ifndef ACCURACYTESTER_H_
#define ACCURACYTESTER_H_

#include <ros/ros.h>
#include <nao_autism_messages/PoseClassification.h>

#include <map>
#include <vector>

namespace classification
{

using namespace std;

class AccuracyTesterModel {
public:
	AccuracyTesterModel();
	void setExpectedClass(int newClass) { _expected_class = newClass; }
	int getExpectedClass() { return _expected_class; }
	float getAccuracy(string learning_name, int classification);

private:
	ros::NodeHandle _nh;
	ros::Subscriber _sub;
	int _expected_class;
	// Map of name of learning method to number of correct predictions for a classification
	map< string, map< int, vector<bool> > > _accuracy_map;

	void classificationCallback(nao_autism_messages::PoseClassification msg);

}; // end class AccuracyTester
}; // end namespace classification


#endif /* ACCURACYTESTER_H_ */
