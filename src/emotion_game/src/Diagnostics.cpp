#include <diagnostic_msgs/DiagnosticArray.h>
#include <ros/ros.h>

ros::Subscriber subscriber;

void callback(diagnostic_msgs::DiagnosticArray msg);

int main(int argc, char** argv)
{
	ros::init(argc, argv, "diagnostic_dumper");

	ros::NodeHandle nh;
	subscriber = nh.subscribe<diagnostic_msgs::DiagnosticArray>("/diagnostics", 1, callback);

	ROS_INFO("Waiting for diagnostic data.");

	ros::spin();

	return 0;
}

#include <iostream>
#include <fstream>

/*
 * byte level # level of operation enumerated above
string name # a description of the test/component reporting
string message # a description of the status
string hardware_id # a hardware unique string
KeyValue[] values # an array of values associated with the status
 */

void callback(diagnostic_msgs::DiagnosticArray msg)
{
	ROS_INFO("Found diagnostic data.");

	std::ofstream fs;
	fs.open("diagnostic_dump.txt", std::ios::out);

	std::vector<diagnostic_msgs::DiagnosticStatus> list = msg.status;
	for (int i=0;i<list.size();i++){
		diagnostic_msgs::DiagnosticStatus& status = list[i];

		fs << "Name: " << status.name << std::endl;
		fs << "Hardware ID: " << status.hardware_id << std::endl;
		fs << "Level: " << static_cast<int>(status.level) << std::endl;
		fs << "Message: " << status.message << std::endl;

		fs << "======================================\n";

		std::vector<diagnostic_msgs::KeyValue> keyValVect = status.values;
		for (int j=0;j<keyValVect.size();j++){
			diagnostic_msgs::KeyValue& keyVal = keyValVect[j];

			fs << "\tKey:" << keyVal.key << std::endl;
			fs << "\tValue:" << keyVal.value << std::endl;
			fs << "--------------------------------------\n";
		}

		fs << "======================================\n";
	}

	fs.flush();
	fs.close();

	ROS_INFO("Data wrote to file, shutting down.");

	//Now we have the diagnostic information, shutdown ros
	ros::shutdown();
}
