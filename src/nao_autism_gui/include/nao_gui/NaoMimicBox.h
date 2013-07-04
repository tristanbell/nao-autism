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

class NaoMimicBox : public QGroupBox
{
	Q_OBJECT

public:
	NaoMimicBox(std::vector<NaoBehavior>& behaviors) : naoControl()
	{
		this->behaviors = behaviors;

		setTitle(MIMIC_BOX_TITLE);
		init();
	}

private:
	nao_control::NaoControl naoControl;

	std::vector<NaoBehavior> behaviors;
	NaoBehavior* currentBehavior;
	NaoBehavior* performedBehavior;

	QPushButton* behaviorBtn;
	QPushButton* promptBtn;
	QPushButton* correctBtn;
	QPushButton* incorrectBtn;

	QComboBox* behaviorBox;
	QLabel* behaviorInfoLabel;

	void init();

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
	void promptButtonPressed();
	void correctButtonPressed();
	void incorrectButtonPressed();

};

}

#endif /* NAOMIMICBOX_H_ */
