/*
 * SvmExport.cpp
 *
 *  Created on: Jul 22, 2013
 *      Author: parallels
 */

#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include <classification/DataPoint.h>
#include <classification/DataLoader.h>
#include <PoseData.h>
#include <classification/PoseDataPoint.h>

#include <ros/ros.h>
#include <boost/foreach.hpp>

using namespace std;

void exportFile(classification::TrainingData &data) {
	/*
	 * Attributes: joints are in order. Attributes chosen are the rotations of each joint.
	 * Output will be: <classification> 1:<head.x>, 2:<head.y>, 3:<head.z>, 4:<head.w>,
	 * 5:<neck.x> ... 60:<right_foot.w>
	 */
	ofstream out("nao_autism", ios::app);

	vector<classification::PoseDataPoint*> pdata =
			classification::PoseDataPoint::convertToPoses(data);

	BOOST_FOREACH(classification::PoseDataPoint* dataPoint, pdata){
		// Classification will be the index, so written first
		out << "+" << dataPoint->getClassification() << " ";

		vector<geometry_msgs::TransformStamped> joints = dataPoint->poseData.getJoints();

		int indexCount = 1;

		for (int i = 0; i < joints.size(); i++) {
			out << indexCount++ << ":" << joints[i].transform.rotation.x << " ";
			out << indexCount++ << ":" << joints[i].transform.rotation.y << " ";
			out << indexCount++ << ":" << joints[i].transform.rotation.z << " ";
			out << indexCount++ << ":" << joints[i].transform.rotation.w << " ";
		}

		out << "\n";
	}

	out.close();
}

int main(int argc, char** argv) {
	if ((argc % 2) == 1 && argc > 1) {
		ros::init(argc, argv, "exporter");

		classification::TrainingData classifiedPoints;

		ROS_INFO("Scanning argument(s)");
		for (unsigned int i = 1; i < argc; i += 2) {
			std::string fileName(argv[i]);
			short classification = strtol(argv[i + 1], NULL, 10);

			std::vector<classification::DataPoint*> dataPoints =
					classification::DataLoader::loadData(fileName);
			for (unsigned int j = 0; j < dataPoints.size(); j++) {
				classification::DataPoint* current = dataPoints[j];

				current->setClassification(classification);
				classifiedPoints.push_back(current);
			}
		}

		exportFile(classifiedPoints);
	} else {
		cerr
				<< "Invalid arguments, The arguments should be supplied in pairs of <filename> and <classification>"
				<< endl;
		cerr
				<< "Where the filename is the name of the bag file containing solely tf transforms and the classification is some short integer."
				<< endl;

		return -1;
	}

	return 0;
}

