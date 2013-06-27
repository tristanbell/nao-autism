#include <NaoGuessBox.h>

#include <QGridLayout>
#include <QComboBox>
#include <QPushButton>

#include <QStringList>
#include <QStringListModel>

#include <iterator>

void NaoGuessBox::init()
{
	QGridLayout* layout = new QGridLayout;

	//Add behavior-based widgets to layout
	QGridLayout* behaviorLayout = new QGridLayout;

	QLabel* behaviorLabel = new QLabel(BEHAVIOR_DROPDOWN_LABEL);
	QComboBox* behaviorBox = new QComboBox;
	addBehaviorsToComboBox(behaviorBox);

	behaviorInfoLabel = new QLabel(BEHAVIOR_INFO_LABEL);

	QPushButton* behaviorPerformButton = new QPushButton("Perform");

	behaviorLayout->addWidget(behaviorLabel, 0, 0);
	behaviorLayout->addWidget(behaviorBox, 0, 1);
	behaviorLayout->addWidget(behaviorInfoLabel, 1, 0, 1, 2);
	behaviorLayout->addWidget(behaviorPerformButton, 2, 1);

	//Add speech-based widgets to layout
	QGridLayout* speechLayout = new QGridLayout;

	QLabel* speechLabel = new QLabel(SPEECH_DROPDOWN_LABEL);
	QComboBox* speechBox = new QComboBox;
	addSpeechToComboBox(speechBox);

	speechInfoLabel = new QLabel(SPEECH_INFO_LABEL);

	QPushButton* speechPerformButton = new QPushButton("Perform");

	speechLayout->addWidget(speechLabel);
	speechLayout->addWidget(speechBox, 0, 1);
	speechLayout->addWidget(speechInfoLabel, 1, 0, 1, 2);
	speechLayout->addWidget(speechPerformButton, 2, 1);

	layout->addItem(behaviorLayout, 0, 0);
	layout->addItem(speechLayout, 1, 0);

	setLayout(layout);
}

void NaoGuessBox::addBehaviorsToComboBox(QComboBox* box)
{
	if (behaviorList != NULL){
		QStringList list;

		for (int i=0;i<behaviorList->size();i++){
			NaoBehavior behavior = behaviorList->at(i);

			list.insert(i, behavior.getQName());
		}

		QStringListModel* model = new QStringListModel(list);
		box->setModel(model);
	}
}

void NaoGuessBox::addSpeechToComboBox(QComboBox* box)
{
	if (speechList != NULL){
		QStringList list;

		for (int i=0;i<speechList->size();i++){
			NaoSpeech speech = speechList->at(i);

			list.insert(i, speech.getQName());
		}

		QStringListModel* model = new QStringListModel(list);
		box->setModel(model);
	}
}
