/*
 * NaoGuessBox.h
 *
 *  Created on: 26 Jun 2013
 *      Author: alex
 */

#ifndef NAOGUESSBOX_H_
#define NAOGUESSBOX_H_

#include <nao_control/NaoControl.h>
#include <NaoBehavior.h>
#include <NaoSpeech.h>

#include <QGroupBox>
#include <QString>
#include <QLabel>
#include <QComboBox>

#include <vector>

namespace nao_gui{

const QString TITLE = "Guess";

const QString BEHAVIOR_DROPDOWN_LABEL = "Behavior:";
const QString BEHAVIOR_INFO_LABEL = "The following behavior will be performed: ";

const QString SPEECH_DROPDOWN_LABEL = "Speech:";
const QString SPEECH_INFO_LABEL = "Nao will say the following: ";

class NaoGuessBox : public QGroupBox
{
	Q_OBJECT

public:
	NaoGuessBox(nao_control::NaoControl* naoControl, std::vector<NaoBehavior>& behaviors)
	{
		this->naoControl=naoControl;
		this->behaviors=behaviors;

		setTitle(TITLE);
		init();
	}

private:
	nao_control::NaoControl* naoControl;

	std::vector<NaoBehavior> behaviors;

	NaoBehavior* currentBehavior;
	NaoSpeech* currentSpeech;

	QComboBox* behaviorBox;
	QLabel* behaviorInfoLabel;

	QComboBox* speechBox;
	QLabel* speechInfoLabel;

	void init();

	void addBehaviorsToComboBox();
	void addSpeechToComboBox(NaoBehavior);

	void setBehaviorInfoLabel(NaoBehavior);
	void setSpeechInfoLabel(NaoSpeech);

public slots:
	//Combobox slots
	void behaviorComboBoxChanged(QString);
	void speechComboBoxChanged(QString);

	//Button slots
	void behaviorButtonClicked();
	void speechButtonClicked();

};

}

#endif /* NAOGUESSBOX_H_ */
