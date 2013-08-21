#include <ros/ros.h>

#include <iostream>

#include <vector>
#include <string>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "nao_diagnostic_node");

	while (ros::ok()){
		std::vector<std::string> stringVector;

		if(ros::master::getNodes(stringVector)){
			std::cout << "Found list of nodes\n";

			for (int i=0;i<stringVector.size();i++){
				std::string& name = stringVector[i];

				std::cout << "Node: " << name << std::endl;
			}
		}

		std::cout << std::endl;
	}

	return 0;
}
