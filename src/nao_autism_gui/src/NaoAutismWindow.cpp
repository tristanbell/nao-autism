#include <nao_gui/NaoAutismWindow.h>
#include <nao_gui/NaoGuessBox.h>
#include <nao_gui/NaoMimicBox.h>
#include <nao_gui/GenericControlBox.h>

#include <QApplication>
#include <QWidget>
#include <QGridLayout>

#include <iostream>

nao_gui::NaoAutismWindow::NaoAutismWindow(std::vector<NaoBehavior>&behaviors, NaoSpeechData& data) : control()
{
	init(behaviors, data);
}

void nao_gui::NaoAutismWindow::init(std::vector<NaoBehavior>& behaviors, NaoSpeechData& data)
{
	//Init qt-based things
	QGridLayout* layout = new QGridLayout;

	NaoGuessBox* guessBox = new NaoGuessBox(&control, behaviors, data);
	NaoMimicBox* mimicBox = new NaoMimicBox(&control, behaviors, data);
	//GenericControlBox* controlBox = new GenericControlBox(&naoControl);

	//Hook up relevant slots and signals
	QObject::connect(guessBox, SIGNAL(gameEnded()),
			this, SLOT(onGuessGameEnd()));

	QObject::connect(mimicBox, SIGNAL(gameEnded()),
			this, SLOT(onMimicGameEnd()));

	QObject::connect(this, SIGNAL(guessGameStart()),
			guessBox, SLOT(onGameStart()));

	QObject::connect(this, SIGNAL(mimicGameStart()),
			mimicBox, SLOT(onGameStart()));

	layout->addWidget(guessBox, 0, 0);
	layout->addWidget(mimicBox, 0, 1);

	setLayout(layout);

	setWindowTitle(WINDOW_TITLE);
	setBaseSize(100, 100);
	setVisible(true);

	emit guessGameStart();
}

void nao_gui::NaoAutismWindow::onGuessGameEnd()
{
	rewardChild();

	emit mimicGameStart();
}

void nao_gui::NaoAutismWindow::onMimicGameEnd()
{
	rewardChild();

	emit guessGameStart();
}

void nao_gui::NaoAutismWindow::rewardChild()
{
	control.say("Lets dance");
	int rnd = 1 + (rand() % MAX_REWARDS);

	std::ostringstream sstream;
	sstream << REWARD_BEHAVIOR_NAME << rnd;
	std::string rewardBehavior = sstream.str();

	control.perform(rewardBehavior);

	control.say("You were great");
	control.perform("clap");
}
