/*
 * poseData.h
 *
 *  Created on: 3 Jul 2013
 *      Author: tristan
 */

#ifndef POSEDATA_H_
#define POSEDATA_H_

#include <geometry_msgs/TransformStamped.h>

struct PoseData {
	geometry_msgs::TransformStamped head;
	geometry_msgs::TransformStamped neck;
	geometry_msgs::TransformStamped torso;
	geometry_msgs::TransformStamped left_shoulder;
	geometry_msgs::TransformStamped left_elbow;
	geometry_msgs::TransformStamped left_hand;
	geometry_msgs::TransformStamped right_shoulder;
	geometry_msgs::TransformStamped right_elbow;
	geometry_msgs::TransformStamped right_hand;
	geometry_msgs::TransformStamped left_hip;
	geometry_msgs::TransformStamped left_knee;
	geometry_msgs::TransformStamped left_foot;
	geometry_msgs::TransformStamped right_hip;
	geometry_msgs::TransformStamped right_knee;
	geometry_msgs::TransformStamped right_foot;
};


#endif /* POSEDATA_H_ */
