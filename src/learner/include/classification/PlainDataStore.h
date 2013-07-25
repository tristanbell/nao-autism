/*
 * PlainDataStore.h
 *
 * Stores training data in a std::vector.
 *
 *  Created on: 10 Jul 2013
 *      Author: tristan
 */
#ifndef PLAINDATASTORE_H_
#define PLAINDATASTORE_H_

#include <classification/DataStore.h>
#include <classification/DataPoint.h>
#include <PoseData.h>
#include <classification/PoseDataPoint.h>
#include <classification/DataLoader.h>
#include <geometry_msgs/TransformStamped.h>

#include <boost/foreach.hpp>
#include <iostream>

namespace classification{

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
//	std::vector<classification::DataPoint*> getDataPoints(const classification::DataPoint *otherPoint,
//			int k) const
//	{
//		if (k > trainingData.size()) {
//			//TODO: Throw error
//			std::cout
//					<< "Hey, k is larger than your training data. Don't do that."
//					<< std::endl;
//		}
//
//		std::vector<classification::DataPoint*> points;
//
//		while (k > 0) {
//			classification::DataPoint *nearest = getDataPoint(otherPoint, points);
//			points.push_back(nearest);
//			k--;
//		}
//
//		return points;
//	}

	std::vector<DataPoint*> getDataPoints() const
	{
		return trainingData;
	}

	std::vector<classification::DataPoint*> getDataPoints(const classification::DataPoint *otherPoint,
			int k) const
	{
		std::list<std::pair<float, DataPoint*> > distToPoint;

		//Construct dist to point list
		for (int i=0;i<trainingData.size();i++){
			DataPoint* p = trainingData[i];

			float dist = otherPoint->getDistance(*p);

			std::pair<float, DataPoint*> pair(dist, p);

			std::list<std::pair<float, DataPoint*> >::iterator it = distToPoint.begin();

			bool inserted = false;
			//Insert in order
			while (it != distToPoint.end()){
				std::pair<float, DataPoint*> current = *it;

				if (dist < current.first){
					//Insert here...
					distToPoint.insert(it, pair);
					inserted = true;
					break;
				}

				it++;
			}

			if (!inserted)
				distToPoint.push_back(pair);
		}

		std::vector<classification::DataPoint*> vect;

		std::list<std::pair<float, DataPoint*> >::iterator it = distToPoint.begin();
		for (int i=0;i<k;i++){
			std::pair<float, DataPoint*> current = *it;
			vect.push_back(current.second);

			std::cout << "Neighbour " << i << " dist: " << current.first << "        \n";

			it++;
		}

		std::cout << "knn_done" << "\n\n";

		return vect;
	}

	/*
	 * Returns true if p is in points. i must be the size of points for the
	 * recursion to work for the whole array.
	 */
	static bool isIn(classification::DataPoint *p, std::vector<classification::DataPoint*> &points, int i)
	{
		if (i <= -1)
			return false;

		return *p == *points[i] || isIn(p, points, i - 1);
	}

};

}

#endif /* PLAINDATASTORE_H_ */
