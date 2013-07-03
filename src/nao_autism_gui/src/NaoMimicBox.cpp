#include <nao_gui/NaoMimicBox.h>

#include <QStringList>
#include <QStringListModel>
#include <QAbstractItemModel>

#include <QGridLayout>

void nao_gui::NaoMimicBox::init()
{
	QGridLayout* layout = new QGridLayout;

	QLabel* behaviorLabel = new QLabel(MIMIC_BEHAVIOR_DROPDOWN_LABEL);

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

	recordTimer = new QTimer;

	startRecordBtn = new QPushButton("Record data");
	startRecordBtn->setEnabled(false);
	QObject::connect(startRecordBtn, SIGNAL(clicked()),
			this, SLOT(recordButtonPressed()));

	layout->addWidget(behaviorLabel, 0, 0);
	layout->addWidget(behaviorBox, 0, 1);
	layout->addWidget(behaviorInfoLabel, 1, 0, 1, 2);
	layout->addWidget(behaviorBtn, 2, 0);
	layout->addWidget(promptBtn, 2, 1);
	layout->addWidget(correctBtn, 3, 0);
	layout->addWidget(incorrectBtn, 3, 1);
	layout->addWidget(startRecordBtn, 4, 0);

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

void nao_gui::NaoMimicBox::promptButtonPressed()
{
	naoControl.say(nao_gui::MIMIC_PROMPT + performedBehavior->getQName().toStdString());

	promptBtn->setEnabled(false);
	behaviorBtn->setEnabled(false);

	startRecordBtn->setEnabled(true);

	correctBtn->setEnabled(true);
	incorrectBtn->setEnabled(true);
}

void nao_gui::NaoMimicBox::correctButtonPressed()
{
	naoControl.say(nao_gui::MIMIC_CORRECT_ANSWER);

	handleAnswerGiven();
}

void nao_gui::NaoMimicBox::incorrectButtonPressed()
{
	naoControl.say(nao_gui::MIMIC_INCORRECT_ANSWER);

	handleAnswerGiven();
}

void nao_gui::NaoMimicBox::handleAnswerGiven()
{
	correctBtn->setEnabled(false);
	incorrectBtn->setEnabled(false);

	behaviorBtn->setEnabled(true);

	startRecordBtn->setEnabled(false);
}

void nao_gui::NaoMimicBox::behaviorButtonClicked()
{
	naoControl.say(MIMIC_PERFORM + currentBehavior->getQName().toStdString());
	naoControl.performWithInit(currentBehavior->getBehaviorName());

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

void nao_gui::NaoMimicBox::recordButtonPressed()
{
	recordTimer->start(20000);

	startRecordBtn->setEnabled(false);
}

void nao_gui::NaoMimicBox::onRecordStop()
{
	if (promptBtn->isEnabled() || correctBtn->isEnabled() || incorrectBtn->isEnabled()){
		startRecordBtn->setEnabled(true);
	}
}
