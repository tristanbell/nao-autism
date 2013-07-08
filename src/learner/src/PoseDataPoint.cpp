#include <PoseDataPoint.h>

float PoseDataPoint::getDistance(const DataPoint& p) const
{
	const PoseDataPoint& otherPoint = dynamic_cast<const PoseDataPoint&>(p);



	return -1;
}

std::vector<geometry_msgs::Quaternion> PoseDataPoint::getRotations()
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
