#include <NaoAutismWindow.h>

#include <NaoGuessBox.h>

#include <QApplication>
#include <QWidget>
#include <QGridLayout>

NaoAutismWindow::NaoAutismWindow(QList<NaoBehavior>* behaviorList, QList<NaoSpeech>* speechList)
{
	init(behaviorList, speechList);
}

void NaoAutismWindow::init(QList<NaoBehavior>* behaviorList, QList<NaoSpeech>* speechList)
{
	//Init qt-based things
	QGridLayout* layout = new QGridLayout;

	NaoGuessBox* box = new NaoGuessBox(behaviorList, speechList);

	layout->addWidget(box);

	setLayout(layout);

	setWindowTitle(WINDOW_TITLE);
	setBaseSize(100, 100);
	setVisible(true);
}
