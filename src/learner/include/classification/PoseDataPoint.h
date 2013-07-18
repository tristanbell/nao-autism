/*
 * PoseDataPoint.h
 *
 *  Created on: 5 Jul 2013
 *      Author: alex
 */

#ifndef POSEDATAPOINT_H_
#define POSEDATAPOINT_H_

#include <PoseData.h>

#include <classification/DataPoint.h>

#include <tf/tfMessage.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Quaternion.h>

#include <iostream>
#include <fstream>

namespace classification{

class PoseDataPoint: public DataPoint
{

public:
	PoseDataPoint(PoseData data) :
			poseData(data)
	{
	}

	// If this is included in the code then whenever you make a PoseDataPoint
	// catkin complains that "call of overloaded ‘PoseDataPoint(PoseData&)’ is ambiguous"
	/*PoseDataPoint(const PoseData& data) :
			classification(-1), poseData(data)
	{

	}*/

	PoseDataPoint(PoseData data, short classification) :
			poseData(data)
	{
		setClassification(classification);
	}

	PoseDataPoint(const PoseData& data, const short& classification) :
		poseData(data)
	{
		setClassification(classification);
	}

	/*
	 * Convert a vector of DataPoint*s to PoseDataPoint*s.
	 */
	static std::vector<classification::PoseDataPoint*> convertToPoses(std::vector<classification::DataPoint*> points)
	{
		std::vector<classification::PoseDataPoint*> conversion;

		for (int i = 0; i < points.size(); i++) {
			classification::PoseDataPoint *p = dynamic_cast<classification::PoseDataPoint*>(points[i]);
			if (p) {
				conversion.push_back(p);
			} else {
				// TODO: throw exception?
			}
		}

		return conversion;
	}

	void serialize(std::string filename)
	{
		poseData.serialize(filename);
	}

	virtual std::vector<float> getPosition() const;
	virtual float getDistance(const DataPoint&) const;
	virtual ros::Time getTimestamp() const;

	const PoseData poseData;
private:

	/**
	 * Get the Quaternion element out of a TransformStamped (just for cleaner code).
	 */
	geometry_msgs::Quaternion extractRotation(
			geometry_msgs::TransformStamped joint) const
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
	static float getDistance(geometry_msgs::Quaternion rotation1,
			geometry_msgs::Quaternion rotation2);

};

//using namespace boost;// {
//using namespace serialization;// {
//
//template<class Archive>
//void serialize(Archive & ar, PoseDataPoint & pd, const unsigned int version)
//{
////	ar & boost::serialization::base_object<DataPoint>(*this);
//	ar & pd.poseData;
//}
//
////} // namespace serialization
////} // namespace boost

} // namespace classification

#endif /* POSEDATAPOINT_H_ */
