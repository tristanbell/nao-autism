/*
 * NaoAutismWindow.h
 *
 *  Created on: 26 Jun 2013
 *      Author: alex
 */

#ifndef NAOAUTISMWINDOW_H_
#define NAOAUTISMWINDOW_H_

#include <nao_control/NaoControl.h>
#include <NaoBehavior.h>
#include <NaoSpeech.h>
#include <NaoSpeechData.h>

#include <QWidget>
#include <QString>

#include <vector>

const int MAX_REWARDS = 3;
const std::string REWARD_BEHAVIOR_NAME = "reward_";

namespace nao_gui{

const QString WINDOW_TITLE = "GUI";

/**
 * This class represents the window that the application runs it, it handles
 * initialising the core 'box' objects that define the sections of the GUI.
 */
class NaoAutismWindow : public QWidget
{

	Q_OBJECT;

public:
	NaoAutismWindow(std::vector<NaoBehavior>&, NaoSpeechData& data);

private:
	nao_control::NaoControl control;

	/**
	 * Called after the constructor has dealt with setting up the correct size/title
	 * of the window. This initialises the core widgets of the application and adds them to
	 * the layout.
	 */
	void init(std::vector<NaoBehavior>&, NaoSpeechData& data);

	void rewardChild();

private Q_SLOTS:
	void onGuessGameEnd();
	void onMimicGameEnd();

Q_SIGNALS:
	void guessGameStart();
	void mimicGameStart();

};

}

#endif /* NAOAUTISMWINDOW_H_ */
