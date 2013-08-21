#include <ros/ros.h>

#include <iostream>
#include <string>

#include <vector>
#include <map>
#include <exception>
#include <iostream>
#include <fstream>

#include <std_msgs/Empty.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>

int _chargeLimit = 10;
int _jointTemperatureLimit = 70;

ros::Subscriber _diagnosticsSubscriber;
ros::Publisher _diagnosticsPublisher;

void diagnosticsCallback(const diagnostic_msgs::DiagnosticArray& msg);

int main(int argc, char** argv)
{
	ros::init(argc, argv, "checker_node");

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

	ros::NodeHandle nh;
	_diagnosticsSubscriber = nh.subscribe("diagnostics", 100, diagnosticsCallback);
	_diagnosticsPublisher = nh.advertise<std_msgs::Empty>("emotion_game/stop", 1);

	ros::spin();

	return 0;
}

void diagnosticsCallback(const diagnostic_msgs::DiagnosticArray& msg)
{
	bool sendStop = false;

	for (int i=0;i<msg.status.size();i++){
		const diagnostic_msgs::DiagnosticStatus& status = msg.status[i];

		if (status.name == "nao_power: Battery"){
			for (int i=0;i<status.values.size();i++){
				const diagnostic_msgs::KeyValue& value = status.values[i];

				if (value.key == "Percentage"){
					float val = strtof(value.value.c_str(), NULL);

					if (val <= _chargeLimit){
						std::cout << "Battery doesn't have enough charge, sending stop message.\n";

						sendStop = true;
						break;
					}
				}
			}
		}else if (status.name.find("Roll") != std::string::npos || status.name.find("Pitch") != std::string::npos || status.name.find("Yaw") != std::string::npos){
			for (int i=0;i<status.values.size();i++){
				const diagnostic_msgs::KeyValue& value = status.values[i];

				if (value.key == "Temperature"){
					float val = strtof(value.value.c_str(), NULL);

					if (val >= _jointTemperatureLimit){
						std::cout << "The following joints: " << status.name << " exceeds the temperature threshold (" << _jointTemperatureLimit << "), sending stop message.\n";

						sendStop = true;
						break;
					}
				}
			}
		}

		if (sendStop){
			std_msgs::Empty emptyMsg;
			_diagnosticsPublisher.publish(emptyMsg);

			break;
		}
	}
}


