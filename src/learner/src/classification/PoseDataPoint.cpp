#include <classification/PoseDataPoint.h>

float classification::PoseDataPoint::getDistance(const DataPoint& p) const
{
	const PoseDataPoint& otherPoint = dynamic_cast<const PoseDataPoint&>(p);

	float sumDistance = 0.0;

	std::vector<geometry_msgs::Quaternion> rotations1 = getRotations();
	std::vector<geometry_msgs::Quaternion> rotations2 = otherPoint.getRotations();

	for (int i = 0; i < rotations1.size(); i++) {
		sumDistance += PoseDataPoint::getDistance(rotations1[i], rotations2[i]);
	}

	return sumDistance;
}

std::vector<float> classification::PoseDataPoint::getPosition() const
{
	std::vector<float> pos;
	std::vector<geometry_msgs::Quaternion> rotations = getRotations();

	for (int i=0;i<rotations.size();i++){
		geometry_msgs::Quaternion current = rotations[i];

		pos.push_back(current.x);
		pos.push_back(current.y);
		pos.push_back(current.z);
		pos.push_back(current.w);
	}

	return pos;
}

float classification::PoseDataPoint::getDistance(geometry_msgs::Quaternion rotation1, geometry_msgs::Quaternion rotation2)
{
	float xDist = std::abs(rotation1.x - rotation2.x);
	float yDist = std::abs(rotation1.y - rotation2.y);
	float zDist = std::abs(rotation1.z - rotation2.z);
	float wDist = std::abs(rotation1.w - rotation2.w);

	return xDist + yDist + zDist + wDist;
}

std::vector<geometry_msgs::Quaternion> classification::PoseDataPoint::getRotations() const
{
	std::vector<geometry_msgs::Quaternion> rotations(15);

	rotations[0] = extractRotation(poseData.head);
	rotations[1] = extractRotation(poseData.neck);
	rotations[2] = extractRotation(poseData.torso);
	rotations[3] = extractRotation(poseData.left_shoulder);
	rotations[4] = extractRotation(poseData.left_elbow);
	rotations[5] = extractRotation(poseData.left_hand);
	rotations[6] = extractRotation(poseData.right_shoulder);
	rotations[7] = extractRotation(poseData.right_elbow);
	rotations[8] = extractRotation(poseData.right_hand);
	rotations[9] = extractRotation(poseData.left_hip);
	rotations[10] = extractRotation(poseData.left_knee);
	rotations[11] = extractRotation(poseData.left_foot);
	rotations[12] = extractRotation(poseData.right_hip);
	rotations[13] = extractRotation(poseData.right_knee);
	rotations[14] = extractRotation(poseData.right_foot);

	return rotations;
}

void classification::PoseDataPoint::writeToFile(rosbag::Bag &bag, ros::Time &time) const
{
	poseData.writeToFile(bag, time);
}

ros::Time classification::PoseDataPoint::getTimestamp() const
{
	return poseData.head.header.stamp;
}




