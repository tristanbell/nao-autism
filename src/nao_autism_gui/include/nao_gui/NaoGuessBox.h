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
	NaoGuessBox(nao_control::NaoControl* control, std::vector<NaoBehavior>& behaviors, NaoSpeechData& speechData)
	{
		this->naoControl=control;
		this->behaviors=behaviors;
		this->data=speechData;

		behaviorPerformed = false;

		setTitle(TITLE);
		init();
	}

private:
	nao_control::NaoControl* naoControl;

	std::vector<NaoBehavior> behaviors;
	NaoSpeechData data;

	/*
	 * Storing pointers to the current behavior and speech means that
	 * no 'search' is required each time the user clicks the button, the search
	 * is done only when the combobox is changed.
	 */
	int currentBehaviorIndex;
	NaoBehavior* currentBehavior;

	int performedBehaviorIndex;
	NaoBehavior* performedBehavior;

	QComboBox* behaviorBox;
	QLabel* behaviorInfoLabel;

	QPushButton* performBehaviorBtn;
	QPushButton* correctBtn;
	QPushButton* tryAgainBtn;
	QPushButton* incorrectBtn;

	QPushButton* endGameBtn;

	bool behaviorPerformed;

	void init();

	/**
	 * This adds the relevant NaoBehaviors inside the vector
	 * into the combobox in the correct manor. It also initialises the
	 * currentBehavior pointer to point to the first entry and
	 * adds the relevant speeches to the combo box for the first NaoBehavior.
	 */
	void addBehaviorsToComboBox();

	/**
	 * Given some NaoBehavior, this function will display the relevant information in the
	 * behavior label (namely, its name).
	 */
	void setBehaviorInfoLabel(const NaoBehavior&);

	void handleAnswer();

	void askQuestion();

	std::string swap(const std::string& base, const std::string& toSwap, const std::string& other);

public Q_SLOTS:
	/*
	 * Combobox slots that get called when the relevant combobox's index
	 * has been changed. The behaviorComboBoxChanged(const QString&) slot will
	 * load the relevant speeches in the speech combobox and update the behavior information
	 * label whilst the speechComboBoxChanged(const QString&) slot will update just the
	 * speech information label.
	 */
	void behaviorComboBoxChanged(const QString&);

	/*
	 * Button slots that get called when the relevant buttons are clicked, the
	 * behaviorButtonClicked() slot will perform the current behavior that the
	 * currentBehavior pointer points to whilst the speechButtonClicked() slot will
	 * perform the current speech that the currentSpeech pointer points to.
	 */
	void behaviorButtonClicked();
	void endButtonClicked();
	void correctButtonClicked();
	void tryAgainButtonClicked();
	void incorrectButtonClicked();

	void onGameStart();

Q_SIGNALS:
	void gameEnded();

};

}

#endif /* NAOGUESSBOX_H_ */
