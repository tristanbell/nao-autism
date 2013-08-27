#include <ros/ros.h>

#include <iostream>
#include <string>

#include <vector>
#include <map>
#include <exception>
#include <iostream>
#include <fstream>

#include <NaoDiagnostics.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "checker_node");

	int _chargeLimit = 10;
	int _jointTemperatureLimit = 70;

	//Check for parameters
	if (argc > 1){
		//There are parameters
		for (int i=1;i<argc;i++){
			if (argv[i][0] == '-' && argv[i][1] == '-'){
				std::string argument(argv[i]);

				if (argument == "--charge"){
					int valIndex = i+1;
					if (valIndex < argc){
						std::cout << "Found --charge argument\n";
						_chargeLimit = strtol(argv[valIndex], NULL, 10);
					}

					i++;
				}else if (argument == "--temperature"){
					int valIndex = i+1;
					if (valIndex < argc){
						std::cout << "Found --temperature argument\n";
						_jointTemperatureLimit = strtol(argv[valIndex], 0, 10);
					}

					i++;
				}else{
					std::cout << "Unknown argument: " << argv[i] << ", perhaps it has been mistyped?\n";
				}
			}else{
				std::cout << "Discarding: " << argv[i] << " it doesn't appear to be an argument.\n";
			}
		}
	}

	NaoDiagnostics naoDiagnostics;

	naoDiagnostics.setBatteryLevelLimit(_chargeLimit);
	naoDiagnostics.setJointTemperatureLimit(_jointTemperatureLimit);

	ros::NodeHandle nh;
	ros::Subscriber subscriber = nh.subscribe("/diagnostics", 1, &NaoDiagnostics::diagnosticMessageCallback, &naoDiagnostics);

	ros::spin();

	return 0;
}
