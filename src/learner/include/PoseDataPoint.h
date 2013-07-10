/*
 * PoseDataPoint.h
 *
 *  Created on: 5 Jul 2013
 *      Author: alex
 */

#ifndef POSEDATAPOINT_H_
#define POSEDATAPOINT_H_

#include <PoseData.h>

#include <DataPoint.h>

#include <tf/tfMessage.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Quaternion.h>

class PoseDataPoint : public DataPoint
{

public:
	PoseDataPoint(PoseData data) : classification(-1)
	{
		poseData = data;
	}

	PoseDataPoint(const PoseData& data) : classification(-1),
										  poseData(data)
	{

	}

	PoseDataPoint(PoseData data, short classification)
	{
		poseData = data;
		this->classification = classification;
	}

	PoseDataPoint(const PoseData& data, const short& classification)
	{
		this->classification = classification;
		data=poseData;
	}

	virtual std::vector<float> getPosition() const;
	virtual float getDistance(const DataPoint&) const;

	short getClassification() const
	{
		return classification;
	}

private:
	const PoseData poseData;
	short classification;

	/**
	 * Get the Quaternion element out of a TransformStamped (just for cleaner code).
	 */
	geometry_msgs::Quaternion extractRotation(geometry_msgs::TransformStamped joint) const
	{
		return joint.transform.rotation;
	}

	/**
	 * Extract all the joint rotations from a PoseData.
	 */
	std::vector<geometry_msgs::Quaternion> getRotations() const;

	/**
	 * Get the sum distance between two Quaternions for nearest-neighbour
	 * calculations.
	 */
	static float getDistance(geometry_msgs::Quaternion rotation1, geometry_msgs::Quaternion rotation2);

	short setClassification(short int classification)
	{
		this->classification = classification;
	}

};

#endif /* POSEDATAPOINT_H_ */
