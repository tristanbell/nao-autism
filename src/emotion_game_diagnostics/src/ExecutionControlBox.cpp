#include <ExecutionControlBox.h>

#include <QGridLayout>

#include <iostream>

void ExecutionControlBox::init()
{
	//Initialise ros subcribers/publishers
	_nodeHandle = ros::NodeHandle();

	_statusSubscriber = _nodeHandle.subscribe("emotion_game/execution_status", 1, &ExecutionControlBox::executionStatusCallback, this);
	_statusPublisher = _nodeHandle.advertise<nao_autism_messages::SetExecutionStatus>("emotion_game/set_execution_status", 1);

	//Initialise GUI elements
	QGridLayout* layout = new QGridLayout;

	_continueBtn = new QPushButton("Continue");
	_continueBtn->setEnabled(false);
	layout->addWidget(_continueBtn, 0, 0);
	QObject::connect(_continueBtn, SIGNAL(clicked()),
			this, SLOT(onContinueBtnClicked()));

	_pauseBtn = new QPushButton("Pause");
	_pauseBtn->setEnabled(false);
	layout->addWidget(_pauseBtn, 0, 1);
	QObject::connect(_pauseBtn, SIGNAL(clicked()),
			this, SLOT(onPauseBtnClicked()));

	_stopBtn = new QPushButton("Stop");
	_stopBtn->setEnabled(false);
	layout->addWidget(_stopBtn, 0, 2);
	QObject::connect(_stopBtn, SIGNAL(clicked()),
			this, SLOT(onStopBtnClicked()));

	setLayout(layout);

	QObject::connect(this, SIGNAL(executionStatusMessageRecieved()),
			this, SLOT(onExecutionStatusMessageRecieved()));

	recievedMsg = 0;
	_statusCheckThread = boost::thread(&ExecutionControlBox::run, this);
}

void ExecutionControlBox::run()
{
	//This method will be run in a separate boost thread, enabling ros to process callbacks
	ros::Rate spinRate(1);
	while (true){
		spinRate.sleep();

		ros::spinOnce();
	}
}

void ExecutionControlBox::executionStatusCallback(nao_autism_messages::ExecutionStatus msg)
{
	if (recievedMsg != 0)
		delete recievedMsg;

	recievedMsg = new nao_autism_messages::ExecutionStatus(msg);

	//Emit a signal to allow a slot to catch it and process the message in the event queue (as all updates to the GUI must be performed here)
	emit executionStatusMessageRecieved();
}

void ExecutionControlBox::onExecutionStatusMessageRecieved()
{
	nao_autism_messages::ExecutionStatus msg = *recievedMsg;

	switch (msg.status){

		case nao_autism_messages::ExecutionStatus::RUNNING:{
			_continueBtn->setEnabled(false);
			_pauseBtn->setEnabled(true);
			_stopBtn->setEnabled(true);

			break;
		}

		case nao_autism_messages::ExecutionStatus::PAUSED:{
			_continueBtn->setEnabled(true);
			_pauseBtn->setEnabled(false);
			_stopBtn->setEnabled(true);

			break;
		}

		case nao_autism_messages::ExecutionStatus::STOPPING:{
			_continueBtn->setEnabled(false);
			_pauseBtn->setEnabled(false);
			_stopBtn->setEnabled(false);

			break;
		}

	}
}

void ExecutionControlBox::onContinueBtnClicked()
{
	//Send continue message
	sendSetStatusMessage(nao_autism_messages::SetExecutionStatus::ACTIVE);
}

void ExecutionControlBox::onPauseBtnClicked()
{
	//Send pause message
	sendSetStatusMessage(nao_autism_messages::SetExecutionStatus::INACTIVE);
}

void ExecutionControlBox::onStopBtnClicked()
{
	//Send stop messages
	sendSetStatusMessage(nao_autism_messages::SetExecutionStatus::STOP);
}

void ExecutionControlBox::sendSetStatusMessage(int status)
{
	nao_autism_messages::SetExecutionStatus msg;
	msg.newStatus = status;

	_statusPublisher.publish(msg);
}

