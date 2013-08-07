#include <PhraseTab.h>

#include <QGridLayout>

const QString PhraseTab::TAB_NAME = "General phrases";

void PhraseTab::init()
{
	QGridLayout* layout = new QGridLayout;
	setLayout(layout);

	_phrasesWidget = new PhrasesWidget;
	layout->addWidget(_phrasesWidget);
}

void PhraseTab::onPhraseGroupLoaded(std::map<QString, PhraseGroupData>& group)
{
	_phrasesWidget->setPhraseGroup(group);
}
