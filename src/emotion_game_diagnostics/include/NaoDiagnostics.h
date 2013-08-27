/*
 * NaoDiagnostics.h
 *
 *  Created on: 27 Aug 2013
 *      Author: alex
 */

#ifndef NAODIAGNOSTICS_H_
#define NAODIAGNOSTICS_H_

#include <ros/ros.h>

#include <std_msgs/Empty.h>
#include <diagnostic_msgs/DiagnosticArray.h>

class NaoDiagnostics
{

public:
	NaoDiagnostics() : _batteryLevelLimit(30),
						_jointTemperatureLimit(60),
						_currentBatteryLevel(-1),
						_avgJointTemperature(-1),
						_nodeHandle()
	{
		_diagnosticsPublisher = _nodeHandle.advertise<std_msgs::Empty>("emotion_game/stop", 1);
	}

	void setBatteryLevelLimit(int batteryLimit);
	int getBatteryLevelLimit() const;

	void setJointTemperatureLimit(int temperatureLimit);
	int getJointTemperatureLimit() const;

	int getCurrentBatteryLevel() const;
	float getAverageJointTemperature() const;

	void diagnosticMessageCallback(const diagnostic_msgs::DiagnosticArray& msg);

private:
	ros::NodeHandle _nodeHandle;
	ros::Publisher _diagnosticsPublisher;

	int _batteryLevelLimit;
	int _jointTemperatureLimit;

	int _currentBatteryLevel;
	float _avgJointTemperature;

};

#endif /* NAODIAGNOSTICS_H_ */
