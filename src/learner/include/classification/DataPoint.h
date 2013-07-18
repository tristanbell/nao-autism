/*
 * DataPoint.h
 *
 *  Created on: 5 Jul 2013
 *      Author: alex
 */

#ifndef DATAPOINT_H_
#define DATAPOINT_H_

#include <vector>
#include <ros/ros.h>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>

namespace classification{

/**
 * This class acts as an interface that distance-based
 * machine learning algorithms will use (such as K-Nearest neighbour).
 */
class DataPoint
{

public:
	virtual ~DataPoint() { }

	void setClassification(short int classification)
	{
		this->classification = classification;
	}

	short getClassification() const
	{
		return classification;
	}

	/**
	 * Given some other DataPoint, this function should calculate
	 * the distance between them.
	 *
	 * Returns: The distance between the two data points.
	 */
	virtual float getDistance(const DataPoint&) const = 0;

	/**
	 * Returns a vector of floats that define its position in n-dimensional
	 * space (n being the size of the returned vector)
	 *
	 * Returns: A vector of floats defining its position in n-dimensional space.
	 */
	virtual std::vector<float> getPosition() const = 0;

	/*
	 * Returns the time this data point was originally created (ros::Time for
	 * now, should be abstracted away from ros later).
	 */
	virtual ros::Time getTimestamp() const = 0;

	/**
	 * Checks whether the two DataPoints are equal.
	 */
	bool operator==(const DataPoint& p){
		return getDistance(p) == 0;
	}

private:

	short classification;

};

} // namespace classification


#endif /* DATAPOINT_H_ */
