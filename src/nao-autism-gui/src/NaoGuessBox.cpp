#include <nao_gui/NaoGuessBox.h>

#include <QGridLayout>
#include <QComboBox>
#include <QPushButton>

#include <QStringList>
#include <QStringListModel>
#include <QAbstractItemModel>

#include <iostream>

#include <ros/ros.h>

void nao_gui::NaoGuessBox::init()
{
	currentBehavior = NULL;
	currentSpeech = NULL;

	QGridLayout* layout = new QGridLayout;

	//Add behavior-based widgets to layout
	QGridLayout* behaviorLayout = new QGridLayout;

	QLabel* behaviorLabel = new QLabel(BEHAVIOR_DROPDOWN_LABEL);
	behaviorBox = new QComboBox;

	behaviorInfoLabel = new QLabel(BEHAVIOR_INFO_LABEL);

	QPushButton* behaviorPerformButton = new QPushButton("Perform");
	behaviorPerformButton->connect(behaviorPerformButton, SIGNAL(clicked()),
			this, SLOT(behaviorButtonClicked()));

	behaviorLayout->addWidget(behaviorLabel, 0, 0);
	behaviorLayout->addWidget(behaviorBox, 0, 1);
	behaviorLayout->addWidget(behaviorInfoLabel, 1, 0, 1, 2);
	behaviorLayout->addWidget(behaviorPerformButton, 2, 1);

	//Add speech-based widgets to layout
	QGridLayout* speechLayout = new QGridLayout;

	QLabel* speechLabel = new QLabel(SPEECH_DROPDOWN_LABEL);
	speechBox = new QComboBox;

	speechInfoLabel = new QLabel(SPEECH_INFO_LABEL);

	QPushButton* speechPerformButton = new QPushButton("Perform");
	speechPerformButton->connect(speechPerformButton, SIGNAL(clicked()),
			this, SLOT(speechButtonClicked()));

	speechLayout->addWidget(speechLabel);
	speechLayout->addWidget(speechBox, 0, 1);
	speechLayout->addWidget(speechInfoLabel, 1, 0, 1, 2);
	speechLayout->addWidget(speechPerformButton, 2, 1);

	layout->addItem(behaviorLayout, 0, 0);
	layout->addItem(speechLayout, 1, 0);

	//Fill behavior and speech combobox
	addBehaviorsToComboBox();

	//Connect behavior and speech box
	behaviorBox->connect(behaviorBox, SIGNAL(currentIndexChanged(const QString&)),
			this, SLOT(behaviorComboBoxChanged(QString)));
	speechBox->connect(speechBox, SIGNAL(currentIndexChanged(const QString&)),
			this, SLOT(speechComboBoxChanged(QString)));

	setLayout(layout);
}

void nao_gui::NaoGuessBox::addBehaviorsToComboBox()
{
	if (behaviors.size() > 0){
		QStringList list;

		currentBehavior = &behaviors[0];

		NaoBehavior current = *currentBehavior;
		setBehaviorInfoLabel(current);

		list.push_back(current.getQName());
		addSpeechToComboBox(current);

		for (int i=1;i<behaviors.size();i++){
			current = behaviors[i];

			list.push_back(current.getQName());
		}

		QStringListModel* model = new QStringListModel(list);
		behaviorBox->setModel(model);
	}
}

void nao_gui::NaoGuessBox::setBehaviorInfoLabel(NaoBehavior behavior)
{
	behaviorInfoLabel->setText(BEHAVIOR_INFO_LABEL + behavior.getQName());
}

void nao_gui::NaoGuessBox::addSpeechToComboBox(NaoBehavior behavior)
{
	std::vector<NaoSpeech> speeches = behavior.getSpeeches();

	if (speeches.size() > 0){
		QStringList list;

		currentSpeech = &speeches[0];

		//Do something with first speech
		NaoSpeech current = *currentSpeech;
		setSpeechInfoLabel(current);

		list.push_back(current.getQName());

		for (int i=1;i<speeches.size();i++){
			current = speeches[i];
			list.push_back(current.getQName());
		}

		QStringListModel* model = new QStringListModel(list);
		speechBox->setModel(model);
	}
}

void nao_gui::NaoGuessBox::setSpeechInfoLabel(NaoSpeech speech)
{
	speechInfoLabel->setText(SPEECH_INFO_LABEL + "\"" + speech.getQSpeech() + "\".");
}

void nao_gui::NaoGuessBox::behaviorComboBoxChanged(QString string)
{
	ROS_INFO("Behavior changed.");

	//Search for behavior object
	for (int i=0;i<behaviors.size();i++){
		NaoBehavior* current = &behaviors[i];

		if (current->getQName() == string){
			currentBehavior = current;
			setBehaviorInfoLabel(*current);

			QAbstractItemModel* oldModel = speechBox->model();

			addSpeechToComboBox(*current);

			oldModel->~QAbstractItemModel();

			break;
		}
	}
}

void nao_gui::NaoGuessBox::speechComboBoxChanged(QString string)
{
	//Search for speech object
	if (currentBehavior != NULL){
		std::vector<NaoSpeech> speeches = currentBehavior->getSpeeches();

		for (int i=0;i<speeches.size();i++){
			NaoSpeech* current = &speeches[i];

			if (current->getQName() == string){
				currentSpeech = current;

				setSpeechInfoLabel(*currentSpeech);

				break;
			}
		}
	}
}

void nao_gui::NaoGuessBox::behaviorButtonClicked()
{
	ROS_INFO("Clicked behavior button.\n");

	if (currentBehavior != NULL)
		naoControl->perform(currentBehavior->getBehaviorName());
}

void nao_gui::NaoGuessBox::speechButtonClicked()
{
	ROS_INFO("Clicked speech button.\n");

	if (currentSpeech != NULL)
		naoControl->say(currentSpeech->getSpeech());
}
