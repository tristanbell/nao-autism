/*
 * GeneralInformationBox.h
 *
 *  Created on: 27 Aug 2013
 *      Author: alex
 */

#ifndef GENERALINFORMATIONBOX_H_
#define GENERALINFORMATIONBOX_H_

#include <NaoDiagnostics.h>

#include <ros/ros.h>
#include <diagnostic_msgs/DiagnosticArray.h>

#include <boost/thread.hpp>

#include <QGroupBox>
#include <QLabel>
#include <QSpinBox>

class GeneralInformationBox : public QGroupBox
{

	Q_OBJECT

public:
	GeneralInformationBox() : QGroupBox()
	{
		init();
	}

private:
	NaoDiagnostics _naoDiagnostics;

	ros::NodeHandle _nodeHandle;
	ros::Subscriber _diagnosticSubscriber;

	boost::thread _diagnosticCheckerThread;

	QLabel* _batteryLevelLabel;
	QLabel* _averageJointTemperatureLabel;

	QSpinBox* _batteryLevelSpinner;
	QSpinBox* _averageTempSpinner;

	void init();

	void run();
	void diagnosticsCallback(const diagnostic_msgs::DiagnosticArray&);

private slots:
	void onUpdate();

	void batteryLevelSpinnerChanged(int value);
	void averageTemperatureSpinnerChanged(int value);

signals:
	void requestUpdate();

};


#endif /* GENERALINFORMATIONBOX_H_ */
