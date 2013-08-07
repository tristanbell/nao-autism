#include <MimicGameTab.h>

#include <QGridLayout>

const QString MimicGameTab::TAB_NAME = "Mimic game";

void MimicGameTab::init()
{
	QGridLayout* layout = new QGridLayout;
	setLayout(layout);

	_phrasesWidget = new PhrasesWidget;
	layout->addWidget(_phrasesWidget, 0, 0);

	//Connect signals to the required slots
	QObject::connect(_phrasesWidget, SIGNAL(currentPhraseGroupIndexChanged(const QString&)),
			this, SLOT(onPhraseGroupBoxIndexChanged(const QString&)));
}

void MimicGameTab::onPhraseGroupLoaded(const std::map<std::string, PhraseGroupData>& phraseGroup)
{
	_phrasesWidget->setPhraseGroup(phraseGroup);
}

void MimicGameTab::onPhraseGroupRetrieved(const PhraseGroupData& data)
{
	_phrasesWidget->setCurrentPhraseGroup(data);
}

void MimicGameTab::onPhraseGroupBoxIndexChanged(const QString& text)
{
	std::string key = text.toStdString();
	emit onPhraseGroupRequired(key);
}
