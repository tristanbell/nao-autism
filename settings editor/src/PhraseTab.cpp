#include <PhraseTab.h>

#include <QGridLayout>

const QString PhraseTab::TAB_NAME = "General phrases";

void PhraseTab::init()
{
	QGridLayout* layout = new QGridLayout;
	setLayout(layout);

	_phrasesWidget = new PhrasesWidget;
	layout->addWidget(_phrasesWidget);

	//Connect signals to the required slots
	QObject::connect(_phrasesWidget, SIGNAL(currentPhraseGroupIndexChanged(const QString&)),
			this, SLOT(onPhraseGroupBoxIndexChanged(const QString&)));
}

void PhraseTab::onPhraseGroupLoaded(const std::map<std::string, PhraseGroupData>& group)
{
	_phrasesWidget->setPhraseGroup(group);
}

void PhraseTab::onPhraseGroupRetrieved(const PhraseGroupData& data)
{
	_phrasesWidget->setCurrentPhraseGroup(data);
}

void PhraseTab::onPhraseGroupBoxIndexChanged(const QString& text)
{
	std::string key = text.toStdString();
	emit onPhraseGroupRequired(key);
}
