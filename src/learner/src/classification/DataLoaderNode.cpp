#include <classification/DataLoader.h>
#include <ros/ros.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "data_loader");

	ROS_INFO("Starting...\n");

//	classification::DataLoader::filterData("timestamps.log");
	classification::TrainingData data = classification::DataLoader::loadData("recordings/_2013-07-24-15-54-06_0.bag");
	ros::Time::init();
	ros::Time start(1374677668.0);
	ros::Time end(1374677688.0);
	ros::Time toWrite = ros::Time::now();
	classification::TrainingData subset = classification::DataLoader::getDataSubset(data, start, end);

	rosbag::Bag standingBag("standing.bag", rosbag::bagmode::Write);

	classification::DataLoader::writeToFile(standingBag, subset, toWrite);

	ROS_INFO("Finished!");

	return 0;
}
