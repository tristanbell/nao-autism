#include <classification/DataLoader.h>
#include <ros/ros.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "data_loader");

	ROS_INFO("Starting...\n");

	classification::DataLoader::filterData("timestamps.log");

	ROS_INFO("Finished!");

	return 0;
}
