/*
 * learner.h
 *
 *  Created on: 3 Jul 2013
 *      Author: tristan
 */

#ifndef LEARNER_H_
#define LEARNER_H_

#include <tf/tfMessage.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Quaternion.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

class Learner
{
public:
	Learner();
	/**
	 * Get the sum distance between two Quaternions for nearest-neighbour
	 * calculations.
	 */
	static float getDistance(geometry_msgs::Quaternion rotation1,
			geometry_msgs::Quaternion rotation2);

};

#endif /* LEARNER_H_ */
