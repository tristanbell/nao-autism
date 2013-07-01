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

		NaoBehavior current = behaviors[0];
		setBehaviorInfoLabel(current);

		currentBehavior = new NaoBehavior(current);

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

		//Do something with first speech
		NaoSpeech current = speeches[0];
		setSpeechInfoLabel(current);

		if (currentSpeech != NULL)
			delete currentSpeech;

		currentSpeech = new NaoSpeech(current);

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
	//Search for behavior object
	for (int i=0;i<behaviors.size();i++){
		NaoBehavior current = behaviors[i];

		if (current.getQName() == string){
			delete currentBehavior;
			currentBehavior = new NaoBehavior(current);

			QAbstractItemModel* oldModel = speechBox->model();

			addSpeechToComboBox(current);

			delete oldModel;

			break;
		}
	}
}

void nao_gui::NaoGuessBox::speechComboBoxChanged(QString string)
{
	//Search for speech object
	if (currentBehavior != NULL){
		const std::vector<NaoSpeech> speeches = currentBehavior->getSpeeches();

		for (int i=0;i<speeches.size();i++){
			const NaoSpeech current = speeches[i];

			if (current.getQName().compare(string) == 0){
				setSpeechInfoLabel(current);

				delete currentSpeech;
				currentSpeech = new NaoSpeech(current);

				break;
			}
		}
	}
}

void nao_gui::NaoGuessBox::behaviorButtonClicked()
{
	if (currentBehavior != NULL)
		naoControl->perform(currentBehavior->getBehaviorName());
}

void nao_gui::NaoGuessBox::speechButtonClicked()
{
	if (currentSpeech != NULL)
		naoControl->say(currentSpeech->getSpeech());
}
