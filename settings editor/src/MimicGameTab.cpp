#include <MimicGameTab.h>

#include <QGridLayout>

const QString MimicGameTab::TAB_NAME = "Mimic game";

void MimicGameTab::init()
{
	QGridLayout* layout = new QGridLayout;
	setLayout(layout);

	_phrasesWidget = new PhrasesWidget;
	layout->addWidget(_phrasesWidget, 0, 0);
}

void MimicGameTab::onPhraseGroupLoaded(std::map<std::string, PhraseGroupData>& phraseGroup)
{
	_phrasesWidget->setPhraseGroup(phraseGroup);
}
