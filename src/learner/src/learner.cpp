/*
 * learner.cpp
 *
 *  Created on: 3 Jul 2013
 *      Author: tristan
 */

#include "learner.h"
#include <PoseData.h>
#include <cmath>

#include <boost/foreach.hpp>

float Learner::getDistance(geometry_msgs::Quaternion rotation1,
		geometry_msgs::Quaternion rotation2)
{
	float xDist = std::abs(rotation1.x - rotation2.x);
	float yDist = std::abs(rotation1.y - rotation2.y);
	float zDist = std::abs(rotation1.z - rotation2.z);
	float wDist = std::abs(rotation1.w - rotation2.w);

	return xDist + yDist + zDist + wDist;
}

float Learner::getDistance(PoseData pose1, PoseData pose2)
{
	float head, neck, torso, left_shoulder, left_elbow, left_hand,
			right_shoulder, right_elbow, right_hand, left_hip, left_knee,
			left_foot, right_hip, right_knee, right_foot;

	float sumDistance = 0.0;

	std::vector<geometry_msgs::Quaternion> rotations1 = Learner::getRotations(
			pose1);
	std::vector<geometry_msgs::Quaternion> rotations2 = Learner::getRotations(
			pose2);

	for (int i = 0; i < rotations1.size(); i++) {
		sumDistance += Learner::getDistance(rotations1[i], rotations2[i]);
	}

	return sumDistance;
}

geometry_msgs::Quaternion Learner::extractRotation(
		geometry_msgs::TransformStamped joint)
{
	return joint.transform.rotation;
}

std::vector<geometry_msgs::Quaternion> Learner::getRotations(PoseData pose)
{
	std::vector<geometry_msgs::Quaternion> rotations(15);

	rotations[0] = extractRotation(pose.head);
	rotations[1] = extractRotation(pose.neck);
	rotations[2] = extractRotation(pose.torso);
	rotations[3] = extractRotation(pose.left_shoulder);
	rotations[4] = extractRotation(pose.left_elbow);
	rotations[5] = extractRotation(pose.left_hand);
	rotations[6] = extractRotation(pose.right_shoulder);
	rotations[7] = extractRotation(pose.right_elbow);
	rotations[8] = extractRotation(pose.right_hand);
	rotations[9] = extractRotation(pose.left_hip);
	rotations[10] = extractRotation(pose.left_knee);
	rotations[11] = extractRotation(pose.left_foot);
	rotations[12] = extractRotation(pose.right_hip);
	rotations[13] = extractRotation(pose.right_knee);
	rotations[14] = extractRotation(pose.right_foot);

	return rotations;
}

