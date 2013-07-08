#include <nao_gui/NaoGuessBox.h>

#include <QGridLayout>
#include <QComboBox>

#include <QStringList>
#include <QStringListModel>
#include <QAbstractItemModel>

#include <iostream>

#include <ros/ros.h>

void nao_gui::NaoGuessBox::init()
{
	currentBehavior = NULL;
	performedBehavior = NULL;

	QGridLayout* layout = new QGridLayout;

	//Add behavior-based widgets to layout
	QLabel* behaviorLabel = new QLabel(BEHAVIOR_DROPDOWN_LABEL);
	behaviorBox = new QComboBox;

	behaviorInfoLabel = new QLabel(BEHAVIOR_INFO_LABEL);

	performBehaviorBtn = new QPushButton("Perform behavior");
	QObject::connect(performBehaviorBtn, SIGNAL(clicked()),
			this, SLOT(behaviorButtonClicked()));

	askQuestionBtn = new QPushButton("Ask question");
	askQuestionBtn->setEnabled(false);
	QObject::connect(askQuestionBtn, SIGNAL(clicked()),
			this, SLOT(askQuestionButtonClicked()));

	correctBtn = new QPushButton("Correct answer");
	correctBtn->setEnabled(false);
	QObject::connect(correctBtn, SIGNAL(clicked()),
			this, SLOT(correctButtonClicked()));

	incorrectBtn = new QPushButton("Incorrect answer");
	incorrectBtn->setEnabled(false);
	QObject::connect(incorrectBtn, SIGNAL(clicked()),
			this, SLOT(incorrectButtonClicked()));

	endGameBtn = new QPushButton("End game");
	QObject::connect(endGameBtn, SIGNAL(clicked()),
			this, SLOT(endButtonClicked()));

	layout->addWidget(behaviorLabel, 0, 0);
	layout->addWidget(behaviorBox, 0, 1);
	layout->addWidget(behaviorInfoLabel, 1, 0, 1, 2);
	layout->addWidget(performBehaviorBtn, 2, 0);
	layout->addWidget(askQuestionBtn, 2, 1);
	layout->addWidget(correctBtn, 3, 0);
	layout->addWidget(incorrectBtn, 3, 1);
	layout->addWidget(endGameBtn, 4, 1);

	//Fill behavior and speech combobox
	addBehaviorsToComboBox();

	//Connect behavior and speech box
	QObject::connect(behaviorBox, SIGNAL(currentIndexChanged(const QString&)),
			this, SLOT(behaviorComboBoxChanged(QString)));

	setLayout(layout);
}

void nao_gui::NaoGuessBox::addBehaviorsToComboBox()
{
	if (behaviors.size() > 0){
		QStringList list;

		NaoBehavior current = behaviors[0];
		setBehaviorInfoLabel(current);

		currentBehavior = new NaoBehavior(current);

		list.push_back(current.getQName());

		for (int i=1;i<behaviors.size();i++){
			current = behaviors[i];

			list.push_back(current.getQName());
		}

		QStringListModel* model = new QStringListModel(list);
		behaviorBox->setModel(model);
	}
}

void nao_gui::NaoGuessBox::setBehaviorInfoLabel(const NaoBehavior& behavior)
{
	behaviorInfoLabel->setText(BEHAVIOR_INFO_LABEL + behavior.getQName());
}

void nao_gui::NaoGuessBox::behaviorComboBoxChanged(const QString& string)
{
	//Search for behavior object
	for (int i=0;i<behaviors.size();i++){
		NaoBehavior current = behaviors[i];

		if (current.getQName() == string){
			setBehaviorInfoLabel(current);

			delete currentBehavior;
			currentBehavior = new NaoBehavior(current);

			break;
		}
	}
}

void nao_gui::NaoGuessBox::endButtonClicked()
{
	behaviorPerformed = false;

	performBehaviorBtn->setEnabled(true);
	askQuestionBtn->setEnabled(false);
	correctBtn->setEnabled(false);
	incorrectBtn->setEnabled(false);

	setEnabled(false);

	if (performedBehavior != NULL)
		delete performedBehavior;

	//Do ending stuff here...
	naoControl->say("Guess the emotion is finished.");

	emit gameEnded();
}

void nao_gui::NaoGuessBox::behaviorButtonClicked()
{
	if (currentBehavior != NULL){
		if (behaviorPerformed){
			naoControl->say(NEXT_EMOTION_SPEECH);
		}else{
			behaviorPerformed = true;
		}

		askQuestionBtn->setEnabled(true);

		if (performedBehavior == NULL){
			performedBehavior = new NaoBehavior(*currentBehavior);
		}

		naoControl->perform(performedBehavior->getBehaviorName());

		askQuestionBtn->setEnabled(true);
	}
}

void nao_gui::NaoGuessBox::askQuestionButtonClicked()
{
	correctBtn->setEnabled(true);
	incorrectBtn->setEnabled(true);
}

void nao_gui::NaoGuessBox::correctButtonClicked()
{
	handleAnswer();
}

void nao_gui::NaoGuessBox::incorrectButtonClicked()
{
	handleAnswer();
}

void nao_gui::NaoGuessBox::handleAnswer()
{
	askQuestionBtn->setEnabled(false);
	correctBtn->setEnabled(false);
	incorrectBtn->setEnabled(false);

	delete performedBehavior;
}

void nao_gui::NaoGuessBox::onGameStart()
{
	naoControl->say("Guess the emotion");

	setEnabled(true);
}
