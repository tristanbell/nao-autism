#include <nao_gui/GenericControlBox.h>

#include <QPushButton>
#include <string>

void nao_gui::GenericControlBox::init()
{
	startRecordBtn = new QPushButton("Start recording");
	QObject::connect(startRecordBtn, SIGNAL(clicked()),
			this, SLOT(startRecordingClicked()));
	startRecordBtn->setEnabled(false);

	endRecordBtn = new QPushButton("Stop recording");
	QObject::connect(endRecordBtn, SIGNAL(clicked()),
			this, SLOT(endRecordingClicked()));
	endRecordBtn->setEnabled(false);

	performPreviousBehaviorBtn = new QPushButton("Perform previous behavior");
	QObject::connect(performPreviousBehaviorBtn, SIGNAL(clicked()),
			this, SLOT(performPreviousBehaviorClicked()));
	performPreviousBehaviorBtn->setEnabled(false);

	performPreviousSpeechBtn = new QPushButton("Perform previous speech");
	QObject::connect(performPreviousSpeechBtn, SIGNAL(clicked()),
			this, SLOT(performPreviousSpeechClicked()));
	performPreviousSpeechBtn->setEnabled(false);

	addWidget(startRecordBtn, 0, 0);
	addWidget(endRecordBtn, 0, 1);
	addWidget(performPreviousBehaviorBtn, 0, 3);
	addWidget(performPreviousSpeechBtn, 0, 4);
}

void nao_gui::GenericControlBox::onBehaviorPerformed()
{
	ROS_INFO("Behavior performed.");

	//Allow previous behavior to be performed
	performPreviousBehaviorBtn->setEnabled(true);

	//Allow recording...
	if (!startRecordBtn->isEnabled() && !endRecordBtn->isEnabled()){
		startRecordBtn->setEnabled(true);
	}
}

void nao_gui::GenericControlBox::onSpeechPerformed()
{
	performPreviousSpeechBtn->setEnabled(true);
}

void nao_gui::GenericControlBox::performPreviousBehaviorClicked()
{
	naoControl->performPreviousBehaviorWithInit();
}

void nao_gui::GenericControlBox::performPreviousSpeechClicked()
{
	naoControl->sayPreviousSpeech();
}

void nao_gui::GenericControlBox::startRecordingClicked()
{
	const std::string previousBehavior = naoControl->getPreviousBehavior();

	if (previousBehavior != ""){
		currentRecorder = RosbagRecorder::record(previousBehavior);

		startRecordBtn->setEnabled(false);
		endRecordBtn->setEnabled(true);
	}
}

void nao_gui::GenericControlBox::endRecordingClicked()
{
	if (currentRecorder != NULL){
		ROS_INFO("Stop.");
		currentRecorder->stop();
		ROS_INFO("Again, stop.");

		startRecordBtn->setEnabled(true);
		endRecordBtn->setEnabled(false);
	}
}
