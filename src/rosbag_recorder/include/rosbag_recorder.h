#ifndef ROSBAG_RECORDER_H
#define ROSBAG_RECORDER_H

#include <ros/ros.h>
#include <rosbag/bag.h>
//#include <tf/transform_listener.h>
#include <tf/tfMessage.h>
#include <string>
#include <sensor_msgs/Image.h>

class RosbagRecorder {
	public:
		static RosbagRecorder *createRecorder(std::string emotionName);
		~RosbagRecorder(void);
		static RosbagRecorder *record(std::string emotionName);
		void stop(void);
		
	private:
		RosbagRecorder(void);
		RosbagRecorder(std::string emotion);
		const std::string currentDateTime(void);
		void init(std::string foldername);
		void recordRaw(void);
		void recordRgb(void);
		void recordPointCloud(void);
		//void recordRotations(void);
		void recordCallback(const tf::tfMessage::ConstPtr& msg);
		void rgbCallback(const sensor_msgs::Image::ConstPtr& msg);
		void pointCloudCallback(const sensor_msgs::Image::ConstPtr& msg);
		
		ros::NodeHandle node;
		//tf::TransformListener listener;
		ros::Rate rate;
		rosbag::Bag *raw_bag;
		//rosbag::Bag *rotations_bag;
		bool stopped;
};

#endif
