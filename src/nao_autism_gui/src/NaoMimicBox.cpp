#include <nao_gui/NaoMimicBox.h>

#include <QStringList>
#include <QStringListModel>
#include <QAbstractItemModel>

#include <QGridLayout>

const char* LOG_FILE_NAME = "timestamps.log";

void nao_gui::NaoMimicBox::init()
{
	QGridLayout* layout = new QGridLayout;

	QLabel* behaviorLabel = new QLabel(MIMIC_BEHAVIOR_DROPDOWN_LABEL);

	startBtn = new QPushButton("Start game");
	QObject::connect(startBtn, SIGNAL(clicked()), this, SLOT(startButtonPressed()));

	endBtn = new QPushButton("End game");
	endBtn->setEnabled(false);
	QObject::connect(endBtn, SIGNAL(clicked()),
			this, SLOT(endButtonPressed()));

	behaviorBox = new QComboBox;
	QObject::connect(behaviorBox, SIGNAL(currentIndexChanged(QString)),
			this, SLOT(behaviorComboBoxChanged(QString)));

	behaviorBtn = new QPushButton("Perform behavior");
	QObject::connect(behaviorBtn, SIGNAL(clicked()),
			this, SLOT(behaviorButtonClicked()));

	behaviorInfoLabel = new QLabel;

	fillBehaviorComboBox();

	promptBtn = new QPushButton("Prompt to mimic");
	QObject::connect(promptBtn, SIGNAL(clicked()),
			this, SLOT(promptButtonPressed()));
	promptBtn->setEnabled(false);

	correctBtn = new QPushButton("Correct answer");
	QObject::connect(correctBtn, SIGNAL(clicked()),
			this, SLOT(correctButtonPressed()));
	correctBtn->setEnabled(false);

	incorrectBtn = new QPushButton("Incorrect answer");
	QObject::connect(incorrectBtn, SIGNAL(clicked()),
			this, SLOT(incorrectButtonPressed()));
	incorrectBtn->setEnabled(false);

	layout->addWidget(startBtn, 0, 0);
	layout->addWidget(endBtn, 0, 1);
	layout->addWidget(behaviorLabel, 1, 0);
	layout->addWidget(behaviorBox, 1, 1);
	layout->addWidget(behaviorInfoLabel, 2, 0, 1, 2);
	layout->addWidget(behaviorBtn, 3, 0);
	layout->addWidget(promptBtn, 3, 1);
	layout->addWidget(correctBtn, 4, 0);
	layout->addWidget(incorrectBtn, 4, 1);

	setLayout(layout);
}

void nao_gui::NaoMimicBox::fillBehaviorComboBox()
{
	if (behaviors.size() > 0){
		QStringList list;

		NaoBehavior current = behaviors[0];
		setBehaviorInfoLabel(current);

		currentBehavior = new NaoBehavior(current);

		for (int i=0;i<behaviors.size();i++){
			current = behaviors[i];

			list.push_back(current.getQName());
		}

		QStringListModel* model = new QStringListModel(list);
		behaviorBox->setModel(model);
	}
}

void nao_gui::NaoMimicBox::setBehaviorInfoLabel(NaoBehavior& behavior)
{
	behaviorInfoLabel->setText(MIMIC_BEHAVIOR_INFO_LABEL + behavior.getQName());
}

void nao_gui::NaoMimicBox::startButtonPressed()
{
	naoControl.say("Guess the emotion is finished.");

	sleep(3);
	rewardChild();

	sleep(3);
	naoControl.say("Copy the robot!");
	naoControl.perform("prompt_2");

	emit mimicGameStarted();

	startBtn->setEnabled(false);
	endBtn->setEnabled(true);
}

void nao_gui::NaoMimicBox::endButtonPressed()
{
	naoControl.say("Copy the robot is finished.");

	sleep(3);
	rewardChild();
}

void nao_gui::NaoMimicBox::promptButtonPressed()
{
	//Write to log file
	writeToLogPrompt();

	naoControl.say(nao_gui::MIMIC_PROMPT + performedBehavior->getQName().toStdString());
	naoControl.perform(nao_gui::MIMIC_PROMPT_BEHAVIOR);

	promptBtn->setEnabled(false);
	behaviorBtn->setEnabled(false);

	correctBtn->setEnabled(true);
	incorrectBtn->setEnabled(true);
}

void nao_gui::NaoMimicBox::correctButtonPressed()
{
	//Write to log file
	writeToLogAnswer(true);

	naoControl.say(nao_gui::MIMIC_CORRECT_ANSWER);
	naoControl.perform(nao_gui::MIMIC_CORRECT_BEHAVIOR);

	handleAnswerGiven();
}

void nao_gui::NaoMimicBox::incorrectButtonPressed()
{
	//write to log file
	writeToLogAnswer(false);

	naoControl.say(nao_gui::MIMIC_INCORRECT_ANSWER);
	naoControl.perform(nao_gui::MIMIC_INCORRECT_BEHAVIOR);

	handleAnswerGiven();
}

void nao_gui::NaoMimicBox::handleAnswerGiven()
{
	correctBtn->setEnabled(false);
	incorrectBtn->setEnabled(false);

	behaviorBtn->setEnabled(true);
}

void nao_gui::NaoMimicBox::behaviorButtonClicked()
{
	naoControl.say(MIMIC_PERFORM + currentBehavior->getQName().toStdString());
	naoControl.performWithInit(currentBehavior->getBehaviorName());

	//Write to log file
	writeToLogBehavior();

	//Point to current behavior
	performedBehavior = currentBehavior;

	promptBtn->setEnabled(true);
}

void nao_gui::NaoMimicBox::behaviorComboBoxChanged(const QString& string)
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

void nao_gui::NaoMimicBox::writeToLogBehavior()
{
	std::ofstream stream;

	stream.open(LOG_FILE_NAME, std::ios::app | std::ios::out);

	stream << "[" << getTimestamp() << "] ";

	stream << "BEHAVIOR_BUTTON ";

	stream << "BEHAVIOR_NAME=" << currentBehavior->getBehaviorName() << ' ';

	stream << "PROMPT_ENABLED=";

	if (promptBtn->isEnabled()){
		stream << "TRUE";
	}else{
		stream << "FALSE";
	}

	stream << '\n';

	stream.close();
}

void nao_gui::NaoMimicBox::writeToLogPrompt()
{
	std::ofstream stream;

	stream.open(LOG_FILE_NAME, std::ios::app | std::ios::out);

	stream << "[" << getTimestamp() << "] ";

	stream << "PROMPT_BUTTON ";

	stream << "BEHAVIOR_NAME=" << currentBehavior->getBehaviorName() << '\n';

	stream.close();
}

void nao_gui::NaoMimicBox::writeToLogAnswer(const bool& ans)
{
	std::ofstream stream;

	stream.open(LOG_FILE_NAME, std::ios::app | std::ios::out);

	stream << "[" << getTimestamp() << "] ";

	if (ans){
		stream << "CORRECT_BUTTON ";
	}else{
		stream << "INCORRECT_BUTTON ";
	}

	stream << "BEHAVIOR_NAME=" << currentBehavior->getBehaviorName() << '\n';

	stream.close();
}

const std::string nao_gui::NaoMimicBox::getTimestamp()
{
	time_t     now = time(0);
	struct tm  tstruct;
	char       buf[70];
	tstruct = *localtime(&now);

	strftime(buf, sizeof(buf), "%d-%m.%X", &tstruct);

	return buf;
}

void nao_gui::NaoMimicBox::rewardChild()
{
	naoControl.say("Lets dance");
	long int rnd = static_cast<int>(((random() / static_cast<float>(RAND_MAX)) * MAX_REWARDS) + 1);

	std::ostringstream sstream;
	sstream << REWARD_BEHAVIOR_NAME << rnd;
	std::string rewardBehavior = sstream.str();

	naoControl.perform(rewardBehavior);
	naoControl.say("You were great");
}
