#include <nao_gui/GenericControlBox.h>

#include <QPushButton>

void nao_gui::GenericControlBox::init()
{
	QPushButton* performPreviousBehaviorBtn = new QPushButton("Perform previous behavior");
	performPreviousBehaviorBtn->connect(performPreviousBehaviorBtn, SIGNAL(clicked()),
			this, SLOT(performPreviousBehaviorClicked()));

	QPushButton* performPreviousSpeechBtn = new QPushButton("Perform previous speech");
	performPreviousSpeechBtn->connect(performPreviousSpeechBtn, SIGNAL(clicked()),
			this, SLOT(performPreviousSpeechClicked()));

	addWidget(performPreviousBehaviorBtn, 0, 0);
	addWidget(performPreviousSpeechBtn, 0, 1);
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
