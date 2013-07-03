/*
 * NaoGuessBox.h
 *
 *  Created on: 26 Jun 2013
 *      Author: alex
 */

#ifndef NAOGUESSBOX_H_
#define NAOGUESSBOX_H_

#include <nao_control/NaoControl.h>
#include <nao_gui/NaoAutismWindow.h>
#include <NaoBehavior.h>
#include <NaoSpeech.h>

#include <QGroupBox>
#include <QString>
#include <QLabel>
#include <QComboBox>
#include <QTimer>
#include <QPushButton>

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
	NaoGuessBox(std::vector<NaoBehavior>& behaviors) : naoControl()
	{
		this->behaviors=behaviors;

		setTitle(TITLE);
		init();
	}

private:
	nao_control::NaoControl naoControl;

	std::vector<NaoBehavior> behaviors;

	const NaoBehavior* currentBehavior;
	const NaoSpeech* currentSpeech;

	QComboBox* behaviorBox;
	QLabel* behaviorInfoLabel;

	QComboBox* speechBox;
	QLabel* speechInfoLabel;

	void init();

	void addBehaviorsToComboBox();
	void addSpeechToComboBox(const NaoBehavior&);

	void setBehaviorInfoLabel(const NaoBehavior&);
	void setSpeechInfoLabel(const NaoSpeech&);

public Q_SLOTS:
	//Combobox slots
	void behaviorComboBoxChanged(const QString&);
	void speechComboBoxChanged(const QString&);

	//Button slots
	void behaviorButtonClicked();
	void speechButtonClicked();

};

}

#endif /* NAOGUESSBOX_H_ */
