/*
 * learner.cpp
 *
 *  Created on: 3 Jul 2013
 *      Author: tristan
 */

#include "learner.h"
#include <cmath>

#include <boost/foreach.hpp>

Learner::Learner()
{
	// TODO Auto-generated constructor stub

}

float Learner::getDistance(geometry_msgs::Quaternion rotation1,
		geometry_msgs::Quaternion rotation2)
{
	float xDist = std::abs(rotation1.x - rotation2.x);
	float yDist = std::abs(rotation1.y - rotation2.y);
	float zDist = std::abs(rotation1.z - rotation2.z);
	float wDist = std::abs(rotation1.w - rotation2.w);

	return xDist + yDist + zDist + wDist;
}
