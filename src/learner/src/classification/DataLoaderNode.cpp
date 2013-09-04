#include <classification/DataLoader.h>
#include <ros/ros.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "data_loader");

	ROS_INFO("Starting...\n");

	if (argc < 2) {
		ROS_INFO("Please supply filename (i.e. 'rosrun learner dataLoader timestamps.log'");
		return 1;
	}

	char *filename = argv[1];

	classification::DataLoader::filterData(filename);

	return 0;
}
