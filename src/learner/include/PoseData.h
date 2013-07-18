/*
 * poseData.h
 *
 *  Created on: 3 Jul 2013
 *      Author: tristan
 */

#ifndef POSEDATA_H_
#define POSEDATA_H_

#include <geometry_msgs/TransformStamped.h>
#include <tf/tfMessage.h>
#include <rosbag/bag.h>
#include <ros/ros.h>
#include <vector>
#include <string>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/foreach.hpp>
#include <iostream>
#include <fstream>

class PoseData
{
public:
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

	std::vector<geometry_msgs::TransformStamped> getJoints() const
	{
		std::vector<geometry_msgs::TransformStamped> joints;

		joints.push_back(head);
		joints.push_back(neck);
		joints.push_back(torso);
		joints.push_back(left_shoulder);
		joints.push_back(left_elbow);
		joints.push_back(left_hand);
		joints.push_back(right_shoulder);
		joints.push_back(right_elbow);
		joints.push_back(right_hand);
		joints.push_back(left_hip);
		joints.push_back(left_knee);
		joints.push_back(left_foot);
		joints.push_back(right_hip);
		joints.push_back(right_knee);
		joints.push_back(right_foot);

		return joints;
	}

	void writeMsg(geometry_msgs::TransformStamped joint, std::vector<geometry_msgs::TransformStamped>& trans,
			tf::tfMessage& msg, rosbag::Bag& bag, ros::Time &time) const
	{
		trans[0] = joint;
		msg.transforms = trans;
		bag.write("/tf", time, msg);
	}

	void writeToFile(rosbag::Bag& bag, ros::Time &time) const
	{
		tf::tfMessage msg;
		std::vector < geometry_msgs::TransformStamped > trans(1);

		writeMsg(head, trans, msg, bag, time);
		writeMsg(neck, trans, msg, bag, time);
		writeMsg(torso, trans, msg, bag, time);
		writeMsg(left_shoulder, trans, msg, bag, time);
		writeMsg(left_elbow, trans, msg, bag, time);
		writeMsg(left_hand, trans, msg, bag, time);
		writeMsg(right_shoulder, trans, msg, bag, time);
		writeMsg(right_elbow, trans, msg, bag, time);
		writeMsg(right_hand, trans, msg, bag, time);
		writeMsg(left_hip, trans, msg, bag, time);
		writeMsg(left_knee, trans, msg, bag, time);
		writeMsg(left_foot, trans, msg, bag, time);
		writeMsg(right_hip, trans, msg, bag, time);
		writeMsg(right_knee, trans, msg, bag, time);
		writeMsg(right_foot, trans, msg, bag, time);
	}
	static PoseData getTestPoseData()
	{
		PoseData test_pose;
		geometry_msgs::TransformStamped transform;
		test_pose.head = transform;
		test_pose.neck = transform;
		test_pose.torso = transform;
		test_pose.left_shoulder = transform;
		test_pose.left_elbow = transform;
		test_pose.left_hand = transform;
		test_pose.right_shoulder = transform;
		test_pose.right_elbow = transform;
		test_pose.right_hand = transform;
		test_pose.left_hip = transform;
		test_pose.left_knee = transform;
		test_pose.left_foot = transform;
		test_pose.right_hip = transform;
		test_pose.right_knee = transform;
		test_pose.right_foot = transform;
		return test_pose;
	}

	void serialize(std::string filename) const
	{
		std::ofstream out;
		out.open(filename.c_str(), std::ios::app | std::ios::out);

//		serializeTransform(out, getJoints());

		std::vector<geometry_msgs::TransformStamped> transforms = getJoints();

		BOOST_FOREACH(geometry_msgs::TransformStamped transform, transforms) {
			out << transform;

		//	out << transform.header.frame_id;
		//	out << transform.header.seq;
		//	out << transform.header.stamp;
		//	out << transform.transform.translation.x;
		//	out << transform.transform.translation.y;
		//	out << transform.transform.translation.z;
		//	out << transform.transform.rotation.x;
		//	out << transform.transform.rotation.y;
		//	out << transform.transform.rotation.z;
		//	out << transform.transform.rotation.w;
		}

		out.close();
	}

//	void serializeTransform(std::ofstream out,
//			const std::vector<geometry_msgs::TransformStamped> &transforms) const
//	{
//		BOOST_FOREACH(geometry_msgs::TransformStamped transform, transforms) {
//			out << transform;
//
////			out << transform.header.frame_id;
////			out << transform.header.seq;
////			out << transform.header.stamp;
////			out << transform.transform.translation.x;
////			out << transform.transform.translation.y;
////			out << transform.transform.translation.z;
////			out << transform.transform.rotation.x;
////			out << transform.transform.rotation.y;
////			out << transform.transform.rotation.z;
////			out << transform.transform.rotation.w;
//		}
//	}
};

//using namespace boost; //{
//using namespace serialization; //{
//
//template<class Archive>
//void serialize(Archive & ar, PoseData & p, const unsigned int version)
//{
//	geometry_msgs::TransformStamped transform = p.head;
//	serializeTransform(ar, transform);
//    transform = p.neck;
//	serializeTransform(ar, transform);
//    transform = p.torso;
//	serializeTransform(ar, transform);
//    transform = p.left_shoulder;
//	serializeTransform(ar, transform);
//    transform = p.left_elbow;
//	serializeTransform(ar, transform);
//    transform = p.left_hand;
//	serializeTransform(ar, transform);
//    transform = p.right_shoulder;
//	serializeTransform(ar, transform);
//    transform = p.right_elbow;
//	serializeTransform(ar, transform);
//    transform = p.right_hand;
//	serializeTransform(ar, transform);
//    transform = p.left_hip;
//	serializeTransform(ar, transform);
//    transform = p.left_knee;
//	serializeTransform(ar, transform);
//    transform = p.left_foot;
//	serializeTransform(ar, transform);
//    transform = p.right_hip;
//	serializeTransform(ar, transform);
//    transform = p.right_knee;
//	serializeTransform(ar, transform);
//    transform = p.right_foot;
//	serializeTransform(ar, transform);
//
//	/*ar & p.head;
//	ar & p.neck;
//	ar & p.torso;
//	ar & p.left_shoulder;
//	ar & p.left_elbow;
//	ar & p.left_hand;
//	ar & p.right_shoulder;
//	ar & p.right_elbow;
//	ar & p.right_hand;
//	ar & p.left_hip;
//	ar & p.left_knee;
//	ar & p.left_foot;
//	ar & p.right_hip;
//	ar & p.right_knee;
//	ar & p.right_foot;*/
//}
//
//template<class Archive>
//void serializeTransform(Archive& ar, geometry_msgs::TransformStamped& transform)
//{
//	ar & transform.header.frame_id;
//	ar & transform.header.seq;
//	ar & transform.header.stamp;
//	ar & transform.transform.translation.x;
//	ar & transform.transform.translation.y;
//	ar & transform.transform.translation.z;
//	ar & transform.transform.rotation.x;
//	ar & transform.transform.rotation.y;
//	ar & transform.transform.rotation.z;
//	ar & transform.transform.rotation.w;
//}

//} // namespace serialization
//} // namespace boost

#endif /* POSEDATA_H_ */
