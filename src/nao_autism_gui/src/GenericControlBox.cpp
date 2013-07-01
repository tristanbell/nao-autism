#include <nao_gui/GenericControlBox.h>

#include <QPushButton>
#include <string>

void nao_gui::GenericControlBox::init()
{
	QPushButton* startRecordBtn = new QPushButton("Start recording");
	startRecordBtn->connect(startRecordBtn, SIGNAL(clicked()),
			this, SLOT(startRecordingClicked()));

	QPushButton* endRecordBtn = new QPushButton("Stop recording");
	endRecordBtn->connect(endRecordBtn, SIGNAL(clicked()),
			this, SLOT(endRecordingClicked()));

	QPushButton* performPreviousBehaviorBtn = new QPushButton("Perform previous behavior");
	performPreviousBehaviorBtn->connect(performPreviousBehaviorBtn, SIGNAL(clicked()),
			this, SLOT(performPreviousBehaviorClicked()));

	QPushButton* performPreviousSpeechBtn = new QPushButton("Perform previous speech");
	performPreviousSpeechBtn->connect(performPreviousSpeechBtn, SIGNAL(clicked()),
			this, SLOT(performPreviousSpeechClicked()));

	addWidget(startRecordBtn, 0, 0);
	addWidget(endRecordBtn, 0, 1);
	addWidget(performPreviousBehaviorBtn, 0, 3);
	addWidget(performPreviousSpeechBtn, 0, 4);
}

void nao_gui::GenericControlBox::performPreviousBehaviorClicked()
{
	ROS_INFO("Test");
	naoControl->performPreviousBehavior();
	ROS_INFO("Hey!");
}

void nao_gui::GenericControlBox::performPreviousSpeechClicked()
{
	naoControl->sayPreviousSpeech();
}

void nao_gui::GenericControlBox::startRecordingClicked()
{
	if (currentRecorder != NULL){
		delete currentRecorder;
	}

	const std::string previousBehavior = naoControl->getPreviousBehavior();

	if (previousBehavior != ""){
		currentRecorder = RosbagRecorder::record(previousBehavior);
	}
}

void nao_gui::GenericControlBox::endRecordingClicked()
{
	if (currentRecorder != NULL){
		currentRecorder->stop();
	}
}
