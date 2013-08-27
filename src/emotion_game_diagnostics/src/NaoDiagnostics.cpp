#include <NaoDiagnostics.h>

#include <iostream>
#include <string>

#include <vector>

#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>

void NaoDiagnostics::setBatteryLevelLimit(int batteryLimit)
{
	_batteryLevelLimit = batteryLimit;
}

int NaoDiagnostics::getBatteryLevelLimit() const
{
	return _batteryLevelLimit;
}

void NaoDiagnostics::setJointTemperatureLimit(int temperatureLimit)
{
	_jointTemperatureLimit = temperatureLimit;
}

int NaoDiagnostics::getJointTemperatureLimit() const
{
	return _jointTemperatureLimit;
}

int NaoDiagnostics::getCurrentBatteryLevel() const
{
	return _currentBatteryLevel;
}

float NaoDiagnostics::getAverageJointTemperature() const
{
	return _avgJointTemperature;
}

void NaoDiagnostics::diagnosticMessageCallback(const diagnostic_msgs::DiagnosticArray& msg)
{
	bool sendStop = false;

	int jointNumber = 0;
	float accumulatedValue;

	for (int i=0;i<msg.status.size();i++){
		const diagnostic_msgs::DiagnosticStatus& status = msg.status[i];

		if (status.name == "nao_power: Battery"){
			for (int i=0;i<status.values.size();i++){
				const diagnostic_msgs::KeyValue& value = status.values[i];

				if (value.key == "Percentage"){
					float val = strtof(value.value.c_str(), NULL);

					_currentBatteryLevel = static_cast<int>(val);

					if (val <= _batteryLevelLimit){
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

					jointNumber++;
					accumulatedValue += val;

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

	_avgJointTemperature = accumulatedValue / static_cast<float>(jointNumber);
	std::cout << "Avg: " << _avgJointTemperature << std::endl;
}
