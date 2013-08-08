#include <Window.h>

#include <GenericPhraseTab.h>

#include <QGridLayout>

void Window::init(boost::shared_ptr<Controller> controller, boost::shared_ptr<Model> model)
{
	setWindowTitle(WINDOW_TITLE);
	setVisible(true);

	QGridLayout* layout = new QGridLayout;
	setLayout(layout);

	_tabs = new QTabWidget;
	layout->addWidget(_tabs);

	//Add tabs
	_baseSettingsTab = new BaseSettingsTab;
	_tabs->addTab(_baseSettingsTab, BaseSettingsTab::TAB_NAME);

	_behaviorTab = new BehaviorTab;
	_tabs->addTab(_behaviorTab, BehaviorTab::TAB_NAME);

	_phrasesTab = new AllPhrasesTab;

	GenericPhraseTab* _generalPhraseTab = _phrasesTab->getGeneralPhraseTab();
	GenericPhraseTab* _guessGamePhraseTab = _phrasesTab->getGuessGamePhraseTab();
	GenericPhraseTab* _mimicGamePhraseTab = _phrasesTab->getMimicGamePhraseTab();

	_tabs->addTab(_phrasesTab, AllPhrasesTab::TAB_NAME);

	//Connect signals and slots so phrase tab can request the controller for a new PhraseGroup
	QObject::connect(_generalPhraseTab, SIGNAL(onPhraseGroupRequired(const std::string&)),
			controller.get(), SLOT(onRequestGeneralPhraseGroup(const std::string&)));
	QObject::connect(model.get(), SIGNAL(generalPhraseGroupRetrieved(const PhraseGroupData&)),
			_generalPhraseTab, SLOT(onPhraseGroupRetrieved(const PhraseGroupData&)));

	QObject::connect(_guessGamePhraseTab, SIGNAL(onPhraseGroupRequired(const std::string&)),
			controller.get(), SLOT(onRequestGuessGamePhraseGroup(const std::string&)));
	QObject::connect(model.get(), SIGNAL(guessGamePhraseGroupRetrieved(const PhraseGroupData&)),
			_guessGamePhraseTab, SLOT(onPhraseGroupRetrieved(const PhraseGroupData&)));

	QObject::connect(_mimicGamePhraseTab, SIGNAL(onPhraseGroupRequired(const std::string&)),
			controller.get(), SLOT(onRequestMimicGamePhraseGroup(const std::string&)));
	QObject::connect(model.get(), SIGNAL(mimicGamePhraseGroupRetrieved(const PhraseGroupData&)),
			_mimicGamePhraseTab, SLOT(onPhraseGroupRetrieved(const PhraseGroupData&)));

	//COnnect signals and slots so that new phrases/phrase behaviors can be added to the model
	QObject::connect(_generalPhraseTab, SIGNAL(onPhraseCreated(std::string&, std::string&)),
			controller.get(), SLOT(onGeneralPhraseCreated(std::string&, std::string&)));
	QObject::connect(_generalPhraseTab, SIGNAL(onPhraseBehaviorCreated(std::string&, std::string&)),
			controller.get(), SLOT(onGeneralPhraseBehaviorCreated(std::string&, std::string&)));

	QObject::connect(_guessGamePhraseTab, SIGNAL(onPhraseCreated(std::string&, std::string&)),
			controller.get(), SLOT(onGuessGamePhraseCreated(std::string&, std::string&)));
	QObject::connect(_guessGamePhraseTab, SIGNAL(onPhraseBehaviorCreated(std::string&, std::string&)),
			controller.get(), SLOT(onGuessGamePhraseBehaviorCreated(std::string&, std::string&)));

	QObject::connect(_mimicGamePhraseTab, SIGNAL(onPhraseCreated(std::string&, std::string&)),
			controller.get(), SLOT(onMimicGamePhraseCreated(std::string&, std::string&)));
	QObject::connect(_mimicGamePhraseTab, SIGNAL(onPhraseBehaviorCreated(std::string&, std::string&)),
			controller.get(), SLOT(onMimicGamePhraseBehaviorCreated(std::string&, std::string&)));

	//Connect signals and slots so the model can alert the view(s) when new PhraseGroupData is loaded
	QObject::connect(model.get(), SIGNAL(generalPhraseGroupLoaded(const std::map<std::string, PhraseGroupData>&)),
			_generalPhraseTab, SLOT(onPhraseGroupLoaded(const std::map<std::string, PhraseGroupData>&)));

	QObject::connect(model.get(), SIGNAL(guessGamePhraseGroupLoaded(const std::map<std::string, PhraseGroupData>&)),
			_guessGamePhraseTab, SLOT(onPhraseGroupLoaded(const std::map<std::string, PhraseGroupData>&)));

	QObject::connect(model.get(), SIGNAL(mimicGamePhraseGroupLoaded(const std::map<std::string, PhraseGroupData>&)),
			_mimicGamePhraseTab, SLOT(onPhraseGroupLoaded(const std::map<std::string, PhraseGroupData>&)));
}
