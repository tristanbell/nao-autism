/*
 * PlainDataStore.cpp
 *
 * Stores training data in a std::vector.
 *
 *  Created on: 10 Jul 2013
 *      Author: tristan
 */

#include <DataStore.h>
#include <DataPoint.h>
#include <PoseData.h>
#include <PoseDataPoint.h>
#include <DataLoader.h>
#include <geometry_msgs/TransformStamped.h>

#include <boost/foreach.hpp>
#include <iostream>

class PlainDataStore: public DataStore
{
public:
	PlainDataStore(TrainingData data)
	{
		this->trainingData = data;
	}

	/*
	 * Gets the closest data point to otherPoint in the training data.
	 */
	DataPoint *getDataPoint(const DataPoint *otherPoint) const
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
	DataPoint *getDataPoint(const DataPoint *otherPoint,
			std::vector<DataPoint*> pointsToIgnore) const
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
	std::vector<DataPoint*> getDataPoints(const DataPoint *otherPoint,
			int k) const
	{
		if (k > trainingData.size()) {
			//TODO: Throw error
			std::cout
					<< "Hey, k is larger than your training data. Don't do that."
					<< std::endl;
		}

		std::vector<DataPoint*> points;

		while (k > 0) {
			DataPoint *nearest = getDataPoint(otherPoint, points);
			points.push_back(nearest);
			k--;
		}

		return points;
	}

	/*
	 * Returns true if p is in points. I must be the size of points for the
	 * recursion to work for the whole array.
	 */
	static bool isIn(DataPoint *p, std::vector<DataPoint*> &points, int i)
	{
		if (i <= -1)
			return false;

		return *p == *points[i] || isIn(p, points, i - 1);
	}

};

/*
 * Convert a vector of DataPoint*s to PoseDataPoint*s.
 */
std::vector<PoseDataPoint*> convertToPoses(std::vector<DataPoint*> points)
{
	std::vector<PoseDataPoint*> conversion;

	for (int i = 0; i < points.size(); i++) {
		PoseDataPoint *p = dynamic_cast<PoseDataPoint*>(points[i]);
		if (p) {
			conversion.push_back(p);
		} else {
			// TODO: throw exception?
		}
	}

	return conversion;
}

int main(int argc, char **argv)
{
	TrainingData poses =
			DataLoader::loadData(
					"/home/tristan/nao-autism/recordings/_2013-07-10-11-38-34_1.bag");

	PlainDataStore store(poses);

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

	PoseDataPoint posePoint(test_pose);
	DataPoint *point1 = &posePoint;
	DataPoint *point2 = store.getDataPoint(point1);

	std::vector<DataPoint*> allThePoints = store.getDataPoints(point1, 5);

//	PoseDataPoint *posey = dynamic_cast<PoseDataPoint*>(point2);
	std::vector<PoseDataPoint*> posey = convertToPoses(allThePoints);

	/*for (int i = 0; i < posey.size(); i++) {
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
}

