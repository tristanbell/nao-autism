/*
 * PoseDataPoint.h
 *
 *  Created on: 5 Jul 2013
 *      Author: alex
 */

#ifndef POSEDATAPOINT_H_
#define POSEDATAPOINT_H_

#include <poseData.h>

#include <DataPoint.h>

#include <tf/tfMessage.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Quaternion.h>

class PoseDataPoint : public DataPoint
{

public:
	PoseDataPoint(PoseData data)
	{
		poseData = data;
	}

	PoseDataPoint(const PoseData& data)
	{
		poseData = data;
	}

	virtual float getDistance(const DataPoint&) const;

private:
	PoseData poseData;

	geometry_msgs::Quaternion extractRotation(geometry_msgs::TransformStamped joint)
	{
		return joint.transform.rotation;
	}

	std::vector<geometry_msgs::Quaternion> getRotations();

};

#endif /* POSEDATAPOINT_H_ */
