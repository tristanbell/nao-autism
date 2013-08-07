#include <GuessGameTab.h>

#include <QGridLayout>

const QString GuessGameTab::TAB_NAME = "Guess game";

void GuessGameTab::init()
{
	QGridLayout* layout = new QGridLayout;
	setLayout(layout);

	_phrasesWidget = new PhrasesWidget;
	layout->addWidget(_phrasesWidget, 0, 0);
}
