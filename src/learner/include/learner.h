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
	/**
	 * Get the sum distance between two Quaternions for nearest-neighbour
	 * calculations.
	 */
	static float getDistance(geometry_msgs::Quaternion rotation1,
			geometry_msgs::Quaternion rotation2);

	/**
	 * Get the sum distance between two PoseDatas for nearest-neighbour
	 * calculations. This only considers the distance between the Quaternion
	 * rotation properties of each joint in a PoseData.
	 */
	static float getDistance(PoseData pose1, PoseData pose2);

private:
	/**
	 * Get the Quaternion element out of a TransformStamped (just for cleaner code).
	 */
	static geometry_msgs::Quaternion extractRotation(geometry_msgs::TransformStamped joint);

	/**
	 * Extract all the joint rotations from a PoseData.
	 */
	static std::vector<geometry_msgs::Quaternion> Learner::getRotations(PoseData pose);

};

#endif /* LEARNER_H_ */
