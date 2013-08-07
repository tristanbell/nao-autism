#include <Window.h>

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

	_phraseTab = new PhraseTab;
	_tabs->addTab(_phraseTab, PhraseTab::TAB_NAME);

	_guessGameTab = new GuessGameTab;
	_tabs->addTab(_guessGameTab, GuessGameTab::TAB_NAME);

	_mimicGameTab = new MimicGameTab;
	_tabs->addTab(_mimicGameTab, MimicGameTab::TAB_NAME);

	//Connect required slots to model signals

	//Connect signals and slots so phrase tab can request the controller for a new PhraseGroup
	QObject::connect(_phraseTab, SIGNAL(onPhraseGroupRequired(const std::string&)),
			controller.get(), SLOT(onRequestGeneralPhraseGroup(const std::string&)));
	QObject::connect(model.get(), SIGNAL(generalPhraseGroupRetrieved(const PhraseGroupData&)),
			_phraseTab, SLOT(onPhraseGroupRetrieved(const PhraseGroupData&)));

	QObject::connect(_guessGameTab, SIGNAL(onPhraseGroupRequired(const std::string&)),
			controller.get(), SLOT(onRequestGuessGamePhraseGroup(const std::string&)));
	QObject::connect(model.get(), SIGNAL(guessGamePhraseGroupRetrieved(const PhraseGroupData&)),
			_guessGameTab, SLOT(onPhraseGroupRetrieved(const PhraseGroupData&)));

	QObject::connect(_mimicGameTab, SIGNAL(onPhraseGroupRequired(const std::string&)),
			controller.get(), SLOT(onRequestMimicGamePhraseGroup(const std::string&)));
	QObject::connect(model.get(), SIGNAL(mimicGamePhraseGroupRetrieved(const PhraseGroupData&)),
			_mimicGameTab, SLOT(onPhraseGroupRetrieved(const PhraseGroupData&)));

	QObject::connect(model.get(), SIGNAL(generalPhraseGroupLoaded(const std::map<std::string, PhraseGroupData>&)),
			_phraseTab, SLOT(onPhraseGroupLoaded(const std::map<std::string, PhraseGroupData>&)));

	QObject::connect(model.get(), SIGNAL(guessGamePhraseGroupLoaded(const std::map<std::string, PhraseGroupData>&)),
			_guessGameTab, SLOT(onPhraseGroupLoaded(const std::map<std::string, PhraseGroupData>&)));

	QObject::connect(model.get(), SIGNAL(mimicGamePhraseGroupLoaded(const std::map<std::string, PhraseGroupData>&)),
			_mimicGameTab, SLOT(onPhraseGroupLoaded(const std::map<std::string, PhraseGroupData>&)));
}
