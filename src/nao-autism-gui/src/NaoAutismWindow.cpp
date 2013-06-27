#include <nao_gui/NaoAutismWindow.h>
#include <nao_gui/NaoGuessBox.h>

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

	NaoGuessBox* box = new NaoGuessBox(&naoControl, behaviors);

	layout->addWidget(box);

	setLayout(layout);

	setWindowTitle(WINDOW_TITLE);
	setBaseSize(100, 100);
	setVisible(true);
}
