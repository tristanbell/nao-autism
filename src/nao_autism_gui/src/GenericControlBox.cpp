#include <nao_gui/GenericControlBox.h>

#include <QPushButton>
#include <string>

void nao_gui::GenericControlBox::init()
{
	recordingTimer = new QTimer;

	startRecordBtn = new QPushButton("Start recording");
	QObject::connect(startRecordBtn, SIGNAL(clicked()),
			this, SLOT(startRecordingClicked()));
	startRecordBtn->setEnabled(false);

	performPreviousBehaviorBtn = new QPushButton("Perform previous behavior");
	QObject::connect(performPreviousBehaviorBtn, SIGNAL(clicked()),
			this, SLOT(performPreviousBehaviorClicked()));
	performPreviousBehaviorBtn->setEnabled(false);

	performPreviousSpeechBtn = new QPushButton("Perform previous speech");
	QObject::connect(performPreviousSpeechBtn, SIGNAL(clicked()),
			this, SLOT(performPreviousSpeechClicked()));
	performPreviousSpeechBtn->setEnabled(false);

	addWidget(startRecordBtn, 0, 0);
	addWidget(performPreviousBehaviorBtn, 0, 2);
	addWidget(performPreviousSpeechBtn, 0, 3);
}

void nao_gui::GenericControlBox::onBehaviorPerformed()
{
	//Allow previous behavior to be performed
	performPreviousBehaviorBtn->setEnabled(true);

	//Allow recording...
	if (!startRecordBtn->isEnabled() && !recordingTimer->isActive()){
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
		//currentRecorder = RosbagRecorder::record(previousBehavior);

		startRecordBtn->setEnabled(false);

		QObject::connect(recordingTimer, SIGNAL(timeout()),
				this, SLOT(onRecordingStopped()));

		recordingTimer->start(20000);
	}
}

void nao_gui::GenericControlBox::onRecordingStopped()
{
	startRecordBtn->setEnabled(true);
}
