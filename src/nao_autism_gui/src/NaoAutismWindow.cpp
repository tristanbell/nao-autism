#include <nao_gui/NaoAutismWindow.h>
#include <nao_gui/NaoGuessBox.h>
#include <nao_gui/NaoMimicBox.h>
#include <nao_gui/GenericControlBox.h>

#include <QApplication>
#include <QWidget>
#include <QGridLayout>

nao_gui::NaoAutismWindow::NaoAutismWindow(std::vector<NaoBehavior>& behaviors)
{
	init(behaviors);
}

void nao_gui::NaoAutismWindow::init(std::vector<NaoBehavior>& behaviors)
{
	//Init qt-based things
	QGridLayout* layout = new QGridLayout;

	NaoGuessBox* guessBox = new NaoGuessBox(behaviors);
	NaoMimicBox* mimicBox = new NaoMimicBox(behaviors);
	//GenericControlBox* controlBox = new GenericControlBox(&naoControl);

	//Hook up relevant slots and signals
	QObject::connect(mimicBox, SIGNAL(mimicGameStarted()),
			guessBox, SLOT(onMimicGameStart()));

	/*
	QObject::connect(guessBox, SIGNAL(behaviorPerformed()),
			controlBox, SLOT(onBehaviorPerformed()));

	QObject::connect(guessBox, SIGNAL(speechPerformed()),
			controlBox, SLOT(onSpeechPerformed()));
	QObject::connect(mimicBox, SIGNAL(speechPerformed()),
			controlBox, SLOT(onSpeechPerformed()));*/

	layout->addWidget(guessBox, 0, 0);
	layout->addWidget(mimicBox, 0, 1);
	//layout->addItem(controlBox, 1, 0);

	setLayout(layout);

	setWindowTitle(WINDOW_TITLE);
	setBaseSize(100, 100);
	setVisible(true);
}
