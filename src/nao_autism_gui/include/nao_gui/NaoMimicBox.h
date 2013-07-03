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

	QTimer* recordTimer;
	QPushButton* startRecordBtn;

	void init();

private:
	void fillBehaviorComboBox();
	void setBehaviorInfoLabel(NaoBehavior&);

	void handleAnswerGiven();

private Q_SLOTS:
	void behaviorComboBoxChanged(const QString&);
	void behaviorButtonClicked();

	void promptButtonPressed();
	void correctButtonPressed();
	void incorrectButtonPressed();

	void recordButtonPressed();
	void onRecordStop();

};

}

#endif /* NAOMIMICBOX_H_ */
