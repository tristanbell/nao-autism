#include <nao_gui/NaoGuessBox.h>

#include <QGridLayout>
#include <QComboBox>

#include <QStringList>
#include <QStringListModel>
#include <QAbstractItemModel>

#include <iostream>

#include <ros/ros.h>

#include <iostream>

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

	correctBtn = new QPushButton("Correct answer");
	correctBtn->setEnabled(false);
	QObject::connect(correctBtn, SIGNAL(clicked()),
			this, SLOT(correctButtonClicked()));

	tryAgainBtn = new QPushButton("Try again");
	tryAgainBtn->setEnabled(false);
	QObject::connect(tryAgainBtn, SIGNAL(clicked()),
			this, SLOT(tryAgainButtonClicked()));

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
	layout->addWidget(correctBtn, 3, 0);
	layout->addWidget(tryAgainBtn, 3, 1);
	layout->addWidget(incorrectBtn, 4, 0);
	layout->addWidget(endGameBtn, 5, 0, 1, 2);

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
		currentBehaviorIndex = 0;

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
			currentBehaviorIndex = i;

			break;
		}
	}
}

void nao_gui::NaoGuessBox::endButtonClicked()
{
	behaviorPerformed = false;

	performBehaviorBtn->setEnabled(true);
	correctBtn->setEnabled(false);
	tryAgainBtn->setEnabled(false);
	incorrectBtn->setEnabled(false);

	setEnabled(false);

	if (performedBehavior != NULL){
		delete performedBehavior;
		performedBehavior = NULL;
	}

	//Do ending stuff here...
	naoControl->say("Guess the emotion is finished.");

	emit gameEnded();
}

void nao_gui::NaoGuessBox::behaviorButtonClicked()
{
	if (currentBehavior != NULL){
		if (behaviorPerformed){
			naoControl->say(data.get("GUESS_NEXT"));
		}else{
			behaviorPerformed = true;
		}

		if (performedBehavior == NULL){
			performedBehavior = new NaoBehavior(*currentBehavior);
			performedBehaviorIndex = currentBehaviorIndex;
		}

		naoControl->perform(performedBehavior->getBehaviorName());

		sleep(3);
		askQuestion();
	}
}

void nao_gui::NaoGuessBox::askQuestion()
{
	correctBtn->setEnabled(true);
	tryAgainBtn->setEnabled(true);
	incorrectBtn->setEnabled(true);

	std::string question = data.get("GUESS_QUESTION");

	int index = 0;

	do{
		index = rand() % behaviors.size();
	}while (index == performedBehaviorIndex);

	int firstPartIndex = question.find("%1");
	int firstPartIndexEnd = firstPartIndex + 2;
	int secondPartIndex = question.find("%2");
	int secondPartIndexEnd = secondPartIndex + 2;

	std::ostringstream ss;

	if (rand() % 2){
		ss << question.substr(0, firstPartIndex) << performedBehavior->getQName().toStdString()
				<< question.substr(firstPartIndexEnd, secondPartIndex - firstPartIndexEnd)
				<< behaviors[index].getQName().toStdString() << question.substr(secondPartIndexEnd);
	}else{
		ss << question.substr(0, firstPartIndex) << behaviors[index].getQName().toStdString()
						<< question.substr(firstPartIndexEnd, secondPartIndex - firstPartIndexEnd)
						<< performedBehavior->getQName().toStdString() << question.substr(secondPartIndexEnd);
	}

	naoControl->say(ss.str());
}

void nao_gui::NaoGuessBox::correctButtonClicked()
{
	std::string speech = swap(data.get("GUESS_CORRECT"), "%1",
			performedBehavior->getQName().toStdString());

	naoControl->say(speech);

	handleAnswer();
}

void nao_gui::NaoGuessBox::tryAgainButtonClicked()
{
	naoControl->say(data.get("GUESS_TRY_AGAIN"));
}

void nao_gui::NaoGuessBox::incorrectButtonClicked()
{
	std::string speech = swap(data.get("GUESS_INCORRECT"), "%1",
			performedBehavior->getQName().toStdString());

	naoControl->say(speech);

	handleAnswer();
}

std::string nao_gui::NaoGuessBox::swap(const std::string& base, const std::string& toSwap,
		const std::string& other)
{
	int index = base.find(toSwap);

	if (index == -1)
		return base;

	std::ostringstream ss;

	ss << base.substr(0, index) << other << base.substr(index + toSwap.size());

	return ss.str();
}

void nao_gui::NaoGuessBox::handleAnswer()
{
	correctBtn->setEnabled(false);
	tryAgainBtn->setEnabled(false);
	incorrectBtn->setEnabled(false);

	delete performedBehavior;
	performedBehavior = NULL;
}

void nao_gui::NaoGuessBox::onGameStart()
{
	naoControl->say("Guess the emotion");
	sleep(1);

	naoControl->say(data.get("GUESS_INTRO_1"));
	naoControl->perform("intro_guess");

	setEnabled(true);
}
