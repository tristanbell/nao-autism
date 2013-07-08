/*
 * NaoMimicBox.h
 *
 *  Created on: 1 Jul 2013
 *      Author: alex
 */

#ifndef NAOMIMICBOX_H_
#define NAOMIMICBOX_H_

#include <nao_control/NaoControl.h>
#include <NaoBehavior.h>

#include <QGridLayout>
#include <QGroupBox>
#include <QString>
#include <QComboBox>
#include <QLabel>
#include <QPushButton>
#include <QTimer>

#include <string>
#include <vector>

#include <iostream>
#include <fstream>

namespace nao_gui{

const QString MIMIC_BOX_TITLE = "Mimic";

const std::string MIMIC_PERFORM = "I am ";
const std::string MIMIC_PROMPT = "Can you show me ";

const std::string MIMIC_CORRECT_ANSWER = "Well done";
const std::string MIMIC_INCORRECT_ANSWER = "Better luck next time";

const QString MIMIC_BEHAVIOR_DROPDOWN_LABEL = "Behavior:";
const QString MIMIC_BEHAVIOR_INFO_LABEL = "The following behavior will be performed: ";

const std::string MIMIC_PROMPT_BEHAVIOR = "prompt_1";
const std::string MIMIC_CORRECT_BEHAVIOR = "right_1";
const std::string MIMIC_INCORRECT_BEHAVIOR = "wrong_1";

const int MAX_REWARDS = 3;
const std::string REWARD_BEHAVIOR_NAME = "reward_";

class NaoMimicBox : public QGroupBox
{
	Q_OBJECT

public:
	NaoMimicBox(nao_control::NaoControl* control, std::vector<NaoBehavior>& behaviors)
	{
		this->naoControl=control;
		this->behaviors = behaviors;

		setTitle(MIMIC_BOX_TITLE);
		init();
	}

private:
	nao_control::NaoControl* naoControl;

	std::vector<NaoBehavior> behaviors;
	NaoBehavior* currentBehavior;
	NaoBehavior* performedBehavior;

	QPushButton* endBtn;
	QPushButton* behaviorBtn;
	QPushButton* promptBtn;
	QPushButton* correctBtn;
	QPushButton* incorrectBtn;

	QComboBox* behaviorBox;
	QLabel* behaviorInfoLabel;

	void init();

	/*
	 * The following writes to the timestamps.log file (this file isn't
	 * binary), the format for each entry is the following:
	 *
	 * [TIMESTAMP] NAME PAREMETER_LIST
	 *
	 * The TIMESTAMP is of the following form: dd-mm.hh:mm:ss
	 *
	 * The NAME is a standard C++ string.
	 *
	 * The PARAMETER_LIST is a space separated list of parameter(s) in the following
	 * form:
	 * 		NAME=VALUE
	 *
	 * The VALUE doesn't contain any type information, however, this will be hard coded into
	 * the loading program as the 'format' isn't meant to be generic (just flexible and easy
	 * to save and load)
	 *
	 * Each entry is separated by the new line character (\n)
	 */

	void writeToLogBehavior();
	void writeToLogPrompt();
	void writeToLogAnswer(const bool& ans);

	const std::string getTimestamp();

private:
	/**
	 * This function will load the necessary NaoBehavior objects stored
	 * inside the NaoBehavior vector into the behavior combobox. It also updates
	 * the behavior information label by calling the setBehaviorInfoLabel(NaoBehavior&) function.
	 */
	void fillBehaviorComboBox();

	/**
	 * Updates the behavior information label to store the correct information about the currently
	 * selected behavior (namely, it's name).
	 */
	void setBehaviorInfoLabel(NaoBehavior&);

	/**
	 * Re-enables the relevant buttons when either the correct or incorrect buttons are clicked.
	 */
	void handleAnswerGiven();

private Q_SLOTS:
	/*
	 * This slot is called when the index in the behavior combobox is changed.
	 * It will change the currentBehavior pointer to point to the currently selected
	 * behavior and update the behavior information label (by calling the setBehaviorInfoLabel(NaoBehavior&)
	 * function where the reference to the NaoBehavior is the currently selected behavior).
	 */
	void behaviorComboBoxChanged(const QString&);

	/*
	 * The following slots are called when the relevant buttons are clicked.
	 * Each one will enable/disable certain buttons to 'guide' the user
	 * through the mimic process. This, hopefully, makes it much easier for the user.
	 */
	void behaviorButtonClicked();
	void endButtonPressed();
	void promptButtonPressed();
	void correctButtonPressed();
	void incorrectButtonPressed();

	void onGameStart();

Q_SIGNALS:
	void gameStarted();
	void gameEnded();

};

}

#endif /* NAOMIMICBOX_H_ */
