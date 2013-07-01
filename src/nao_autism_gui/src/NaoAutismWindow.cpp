#include <nao_gui/NaoAutismWindow.h>
#include <nao_gui/NaoGuessBox.h>
#include <nao_gui/NaoMimicBox.h>
#include <nao_gui/GenericControlBox.h>

#include <QApplication>
#include <QWidget>
#include <QGridLayout>

nao_gui::NaoAutismWindow::NaoAutismWindow(std::vector<NaoBehavior>& behaviors)
	: naoControl()
{
	init(behaviors);
}

void nao_gui::NaoAutismWindow::init(std::vector<NaoBehavior>& behaviors)
{
	//Init qt-based things
	QGridLayout* layout = new QGridLayout;

	NaoGuessBox* guessBox = new NaoGuessBox(&naoControl, behaviors);
	NaoMimicBox* mimicBox = new NaoMimicBox(&naoControl);
	GenericControlBox* controlBox = new GenericControlBox(&naoControl);

	layout->addWidget(guessBox, 0, 0);
	layout->addWidget(mimicBox, 0, 1);
	layout->addItem(controlBox, 1, 0);

	setLayout(layout);

	setWindowTitle(WINDOW_TITLE);
	setBaseSize(100, 100);
	setVisible(true);
}
