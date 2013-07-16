/*
 * PlainDataStore.cpp
 *
 * Stores training data in a std::vector.
 *
 *  Created on: 10 Jul 2013
 *      Author: tristan
 */

#include <classification/DataStore.h>
#include <classification/DataPoint.h>
#include <PoseData.h>
#include <classification/PoseDataPoint.h>
#include <classification/DataLoader.h>
#include <geometry_msgs/TransformStamped.h>

#include <boost/foreach.hpp>
#include <iostream>

class PlainDataStore: public classification::DataStore
{
public:
	PlainDataStore(classification::TrainingData data)
	{
		this->trainingData = data;
	}

	/*
	 * Gets the closest data point to otherPoint in the training data.
	 */
	classification::DataPoint *getDataPoint(const classification::DataPoint *otherPoint) const
	{
		float shortestDistance = otherPoint->getDistance(*trainingData[0]);
		int resultIndex = 0;

		for (int i = 1; i < trainingData.size(); i++) {
			float thisDistance = otherPoint->getDistance(*trainingData[i]);

			if (thisDistance < shortestDistance) {
				shortestDistance = thisDistance;
				resultIndex = i;
			}
		}

		return trainingData[resultIndex];
	}

	/*
	 * Gets the closest data point in the training data that isn't in pointsToIgnore.
	 */
	classification::DataPoint *getDataPoint(const classification::DataPoint *otherPoint,
			std::vector<classification::DataPoint*> pointsToIgnore) const
	{
		float shortestDistance = otherPoint->getDistance(*trainingData[0]);
		int resultIndex = 0;

		for (int i = 1; i < trainingData.size(); i++) {
			float thisDistance = otherPoint->getDistance(*trainingData[i]);

			// If the shortest distance point is in pointsToIgnore, return the next shortest point
			if (thisDistance < shortestDistance
					&& !isIn(trainingData[i], pointsToIgnore,
							pointsToIgnore.size() - 1)) {
				shortestDistance = thisDistance;
				resultIndex = i;
			}
		}

		printf("Distance to point: %f\n", shortestDistance);

		return trainingData[resultIndex];
	}

	/*
	 * Gets k closest DataPoints to otherPoint.
	 */
	std::vector<classification::DataPoint*> getDataPoints(const classification::DataPoint *otherPoint,
			int k) const
	{
		if (k > trainingData.size()) {
			//TODO: Throw error
			std::cout
					<< "Hey, k is larger than your training data. Don't do that."
					<< std::endl;
		}

		std::vector<classification::DataPoint*> points;

		while (k > 0) {
			classification::DataPoint *nearest = getDataPoint(otherPoint, points);
			points.push_back(nearest);
			k--;
		}

		return points;
	}

	/*
	 * Returns true if p is in points. I must be the size of points for the
	 * recursion to work for the whole array.
	 */
	static bool isIn(classification::DataPoint *p, std::vector<classification::DataPoint*> &points, int i)
	{
		if (i <= -1)
			return false;

		return *p == *points[i] || isIn(p, points, i - 1);
	}

};

/*
 * Convert a vector of DataPoint*s to PoseDataPoint*s.
 */
std::vector<classification::PoseDataPoint*> convertToPoses(std::vector<classification::DataPoint*> points)
{
	std::vector<classification::PoseDataPoint*> conversion;

	for (int i = 0; i < points.size(); i++) {
		classification::PoseDataPoint *p = dynamic_cast<classification::PoseDataPoint*>(points[i]);
		if (p) {
			conversion.push_back(p);
		} else {
			// TODO: throw exception?
		}
	}

	return conversion;
}

/*int main(int argc, char **argv)
{
	std::vector<std::string> ts = classification::DataLoader::readTimestampFile("timestamps.log");
	for (int i = 0; i < ts.size(); i++) {
		std::cout << ts[i] << std::endl;
	}

	/*
	// Test timestamp taken from log file
	std::string timestamp =
			"[1373453545.520850339] BEHAVIOR_BUTTON BEHAVIOR_NAME=happy_1 PROMPT_ENABLED=FALSE\n[1373453575.833215096] PROMPT_BUTTON BEHAVIOR_NAME=happy_1\n[1373453581.945690118] CORRECT_BUTTON BEHAVIOR_NAME=happy_1";

	// Get the start and end times for the timestamp
	ros::Time::init();
	ros::Time start;
	ros::Time end;
	std::string behavior;
	classification::DataLoader::parseTimestamp(timestamp, start, end, behavior);

	std::cout << std::endl << "Start: " << start << ", End: " << end << std::endl;
	std::cout << behavior << std::endl << std::endl;

	// Find the file that contains the timestamp
	std::string filepath = classification::DataLoader::findFile("/home/tristan/nao-autism/recordings/", start.toBoost() + boost::posix_time::hours(1));
	std::cout << filepath << std::endl;

	// Construct training data from that file
	classification::TrainingData poses =
			classification::DataLoader::loadData(filepath);

	// Create a test PoseData object to compare against (all 0s)
	PoseData test_pose;
	geometry_msgs::TransformStamped transform;
	test_pose.head = transform;
	test_pose.neck = transform;
	test_pose.torso = transform;
	test_pose.left_shoulder = transform;
	test_pose.left_elbow = transform;
	test_pose.left_hand = transform;
	test_pose.right_shoulder = transform;
	test_pose.right_elbow = transform;
	test_pose.right_hand = transform;
	test_pose.left_hip = transform;
	test_pose.left_knee = transform;
	test_pose.left_foot = transform;
	test_pose.right_hip = transform;
	test_pose.right_knee = transform;
	test_pose.right_foot = transform;

	classification::PoseDataPoint posePoint(test_pose);
	classification::DataPoint *point1 = &posePoint;
//	DataPoint *point2 = store.getDataPoint(point1);

	// Get the subset of data that corresponds to the timestamp
	classification::TrainingData subset = classification::DataLoader::getDataSubset(poses, start, end);

	// Create a PlainDataStore from that subset
	PlainDataStore store(subset);
	// Get the 5 nearest neighbours to the test point
	classification::TrainingData allThePoints = store.getDataPoints(point1, 5);

//	std::vector<PoseDataPoint*> posey = convertToPoses(subset);
    */

	/*for (int i = 0; i < subset.size(); i++) {
		std::cout << posey[i]->poseData.head << std::endl;
		std::cout << posey[i]->poseData.neck << std::endl;
		std::cout << posey[i]->poseData.torso << std::endl;
		std::cout << posey[i]->poseData.left_shoulder << std::endl;
		std::cout << posey[i]->poseData.left_elbow << std::endl;
		std::cout << posey[i]->poseData.left_hand << std::endl;
		std::cout << posey[i]->poseData.right_shoulder << std::endl;
		std::cout << posey[i]->poseData.right_elbow << std::endl;
		std::cout << posey[i]->poseData.right_hand << std::endl;
		std::cout << posey[i]->poseData.left_hip << std::endl;
		std::cout << posey[i]->poseData.left_knee << std::endl;
		std::cout << posey[i]->poseData.left_foot << std::endl;
		std::cout << posey[i]->poseData.right_hip << std::endl;
		std::cout << posey[i]->poseData.right_knee << std::endl;
		std::cout << posey[i]->poseData.right_foot << std::endl;
	}*/
//}

