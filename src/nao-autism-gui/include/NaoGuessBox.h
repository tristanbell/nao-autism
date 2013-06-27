/*
 * NaoGuessBox.h
 *
 *  Created on: 26 Jun 2013
 *      Author: alex
 */

#ifndef NAOGUESSBOX_H_
#define NAOGUESSBOX_H_

#include <NaoBehavior.h>
#include <NaoSpeech.h>

#include <QGroupBox>
#include <QString>
#include <QLabel>
#include <QList>
#include <QComboBox>

const QString TITLE = "Guess";

const QString BEHAVIOR_DROPDOWN_LABEL = "Behavior:";
const QString BEHAVIOR_INFO_LABEL = "The following behavior will be performed: ";

const QString SPEECH_DROPDOWN_LABEL = "Speech:";
const QString SPEECH_INFO_LABEL = "Nao will say the following: ";

class NaoGuessBox : public QGroupBox
{

public:
	NaoGuessBox(QList<NaoBehavior>* behaviorList, QList<NaoSpeech>* speechList)
	{
		this->behaviorList = behaviorList;
		this->speechList = speechList;

		setTitle(TITLE);

		init();
	}

private:
	QList<NaoBehavior>* behaviorList;
	QList<NaoSpeech>* speechList;

	QLabel* behaviorInfoLabel;
	QLabel* speechInfoLabel;

	void init();

	void addBehaviorsToComboBox(QComboBox*);
	void addSpeechToComboBox(QComboBox* box);

};

#endif /* NAOGUESSBOX_H_ */
