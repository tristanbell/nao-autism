#include <Window.h>

#include <GenericPhraseTab.h>
#include <GameBehaviorsTab.h>

#include <QMenuBar>
#include <QMenu>
#include <QGridLayout>

void Window::init(boost::shared_ptr<Controller> controller, boost::shared_ptr<Model> model)
{
	setWindowTitle(WINDOW_TITLE);
	setVisible(true);

	QWidget* layoutWidget = new QWidget;

	QGridLayout* layout = new QGridLayout;
	layoutWidget->setLayout(layout);

	setCentralWidget(layoutWidget);

	//Setup menu bar
	QMenuBar* menuBar = new QMenuBar(this);
	setMenuBar(menuBar);

	_fileMenu = new FileMenu;
	menuBar->addMenu(_fileMenu);

	_tabs = new QTabWidget;
	layout->addWidget(_tabs);

	//Add tabs
	_baseSettingsTab = new BaseSettingsTab;
	_tabs->addTab(_baseSettingsTab, BaseSettingsTab::TAB_NAME);

	_behaviorsTab = new BehaviorsTab;
	_tabs->addTab(_behaviorsTab, BehaviorsTab::TAB_NAME);

	GameBehaviorsTab* gameBehaviorTab = _behaviorsTab->getGameBehaviorsTab();

	_phrasesTab = new PhrasesTab;

	GenericPhraseTab* _generalPhraseTab = _phrasesTab->getGeneralPhraseTab();
	GenericPhraseTab* _guessGamePhraseTab = _phrasesTab->getGuessGamePhraseTab();
	GenericPhraseTab* _mimicGamePhraseTab = _phrasesTab->getMimicGamePhraseTab();

	_tabs->addTab(_phrasesTab, PhrasesTab::TAB_NAME);

	//Connect signals to allow saving and loading of data
	QObject::connect(_fileMenu, SIGNAL(onOpenRequested(const std::string&)),
			controller.get(), SLOT(onOpenRequested(const std::string&)));

	QObject::connect(model.get(), SIGNAL(successfulOpen(const std::string&)),
			_fileMenu, SLOT(onSuccessfulOpen(const std::string&)));
	QObject::connect(model.get(), SIGNAL(unsuccessfulOpen(const std::string&)),
			_fileMenu, SLOT(onUnsuccessfulOpen(const std::string&)));

	QObject::connect(_fileMenu, SIGNAL(onSaveRequested()),
			controller.get(), SLOT(onSaveRequested()));
	QObject::connect(_fileMenu, SIGNAL(onSaveAsRequested(const std::string&)),
			controller.get(), SLOT(onSaveAsRequested(const std::string&)));

	QObject::connect(model.get(), SIGNAL(successfulSave(const std::string&)),
			_fileMenu, SLOT(onSuccessfulSave(const std::string&)));
	QObject::connect(model.get(), SIGNAL(unsuccessfulSave(const std::string&)),
			_fileMenu, SLOT(onUnsuccessfulSave(const std::string&)));

	//Connect signals and slots to allow for loading of settings
	QObject::connect(model.get(), SIGNAL(baseSettingsLoaded(const BaseSettingsData&)),
			_baseSettingsTab, SLOT(onSettingsLoaded(const BaseSettingsData&)));

	//Connect signals and slots to allow for updating of settings
	QObject::connect(_baseSettingsTab, SIGNAL(onSettingsUpdated(const BaseSettingsData&)),
			controller.get(), SLOT(onBaseSettingsUpdated(const BaseSettingsData&)));

	//Connect signals and slots so phrase tab can request the controller to retrieve a particular PhraseGroup
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

	//Connect signals and slots so behavior tab can request the controller for a particular BehaviorData
	QObject::connect(gameBehaviorTab, SIGNAL(behaviorDataRequired(const std::string&)),
			controller.get(), SLOT(onRequestGameBehaviors(const std::string&)));
	QObject::connect(model.get(), SIGNAL(gameBehaviorRetrieved(const BehaviorData&)),
			gameBehaviorTab, SLOT(onBehaviorDataRetrieved(const BehaviorData&)));

	//Connect signals and slots so that new phrases/phrase behaviors can be added to the model
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

	//Connect signals and slots so that new phrases/phrase behaviors can be removed from the model
	QObject::connect(_generalPhraseTab, SIGNAL(onPhraseRemoved(const std::string&, const std::string&)),
			controller.get(), SLOT(onGeneralPhraseRemoved(const std::string&, const std::string&)));
	QObject::connect(_generalPhraseTab, SIGNAL(onPhraseBehaviorRemoved(const std::string&, const std::string&)),
			controller.get(), SLOT(onGeneralPhraseBehaviorRemoved(const std::string&, const std::string&)));

	//Connect signals and slots so that new behaviors can be added to the model
	QObject::connect(gameBehaviorTab, SIGNAL(onBehaviorCreated(const std::string&, const std::string&)),
			controller.get(), SLOT(onGameBehaviorCreated(const std::string&, const std::string&)));
	QObject::connect(gameBehaviorTab, SIGNAL(onBehaviorRemoved(const std::string&, const std::string&)),
			controller.get(), SLOT(onGameBehaviorRemoved(const std::string&, const std::string&)));

	//Connect signals and slots so the model can alert the view(s) when new PhraseGroupData is loaded
	QObject::connect(model.get(), SIGNAL(generalPhraseGroupLoaded(const std::map<std::string, PhraseGroupData>&)),
			_generalPhraseTab, SLOT(onPhraseGroupLoaded(const std::map<std::string, PhraseGroupData>&)));

	QObject::connect(model.get(), SIGNAL(guessGamePhraseGroupLoaded(const std::map<std::string, PhraseGroupData>&)),
			_guessGamePhraseTab, SLOT(onPhraseGroupLoaded(const std::map<std::string, PhraseGroupData>&)));

	QObject::connect(model.get(), SIGNAL(mimicGamePhraseGroupLoaded(const std::map<std::string, PhraseGroupData>&)),
			_mimicGamePhraseTab, SLOT(onPhraseGroupLoaded(const std::map<std::string, PhraseGroupData>&)));

	//Connect signals and slots so the model can alert the view(s) when new BehaviorData is loaded
	QObject::connect(model.get(), SIGNAL(gameBehaviorsLoaded(const std::list<BehaviorData>&)),
			gameBehaviorTab, SLOT(onBehaviorListLoaded(const std::list<BehaviorData>&)));
}
