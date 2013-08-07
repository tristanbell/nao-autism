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

void PhraseTab::onPhraseGroupLoaded(std::map<std::string, PhraseGroupData>& group)
{
	_phrasesWidget->setPhraseGroup(group);
}

void PhraseTab::onPhraseGroupBoxIndexChanged(const QString& text)
{
	Controller* cntrl = _controllerPtr.get();

	//Sanity check
	if (cntrl != NULL){
		std::string key = text.toStdString();

		const PhraseGroupData& data = cntrl->getGeneralPhraseGroup(key);
		_phrasesWidget->setCurrentPhraseGroup(data);
	}
}
