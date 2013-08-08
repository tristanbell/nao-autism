#include <GenericPhraseTab.h>

#include <QGridLayout>

const QString GenericPhraseTab::TAB_NAME = "General phrases";

void GenericPhraseTab::init()
{
	QGridLayout* layout = new QGridLayout;
	setLayout(layout);

	_phrasesWidget = new PhrasesWidget;
	layout->addWidget(_phrasesWidget);

	//Connect signals to the required slots
	QObject::connect(_phrasesWidget, SIGNAL(currentPhraseGroupIndexChanged(const QString&)),
			this, SLOT(onPhraseGroupBoxIndexChanged(const QString&)));
}

QString GenericPhraseTab::getTabName() const
{
	return _tabName;
}

void GenericPhraseTab::phraseCreated(std::string& key, std::string& phrase)
{
	emit onPhraseCreated(key, phrase);
}

void GenericPhraseTab::phraseBehaviorCreated(std::string& key, std::string& phrase)
{
	emit onPhraseBehaviorCreated(key, phrase);
}

void GenericPhraseTab::onPhraseGroupLoaded(const std::map<std::string, PhraseGroupData>& group)
{
	_phrasesWidget->setPhraseGroup(group);
}

void GenericPhraseTab::onPhraseGroupRetrieved(const PhraseGroupData& data)
{
	_phrasesWidget->setCurrentPhraseGroup(data);
}

void GenericPhraseTab::onPhraseGroupBoxIndexChanged(const QString& text)
{
	std::string key = text.toStdString();
	emit onPhraseGroupRequired(key);
}
