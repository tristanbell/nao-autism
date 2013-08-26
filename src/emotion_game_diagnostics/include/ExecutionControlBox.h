/*
 * ExecutionControlBox.h
 *
 *  Created on: 26 Aug 2013
 *      Author: rapid
 */

#ifndef EXECUTIONCONTROLBOX_H_
#define EXECUTIONCONTROLBOX_H_

#include <nao_autism_messages/ExecutionStatus.h>
#include <nao_autism_messages/SetExecutionStatus.h>

#include <ros/ros.h>

#include <QGroupBox>
#include <QPushButton>

#include <boost/thread.hpp>

class ExecutionControlBox : public QGroupBox
{

	Q_OBJECT

public:
	ExecutionControlBox() : QGroupBox()
	{
		init();
	}

private:
	ros::NodeHandle _nodeHandle;
	ros::Subscriber _statusSubscriber;
	ros::Publisher _statusPublisher;

	nao_autism_messages::ExecutionStatus* recievedMsg;

	boost::thread _statusCheckThread;
	//boost::mutex _mtx;

	QPushButton* _continueBtn;
	QPushButton* _pauseBtn;
	QPushButton* _stopBtn;

	void init();

	void run();
	void executionStatusCallback(nao_autism_messages::ExecutionStatus);

	void sendSetStatusMessage(int status);

signals:
	void executionStatusMessageRecieved();

private slots:
	void onExecutionStatusMessageRecieved();

	void onContinueBtnClicked();
	void onPauseBtnClicked();
	void onStopBtnClicked();

};


#endif /* EXECUTIONCONTROLBOX_H_ */
