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

	_phraseTab = new PhraseTab(controller);
	_tabs->addTab(_phraseTab, PhraseTab::TAB_NAME);

	_guessGameTab = new GuessGameTab;
	_tabs->addTab(_guessGameTab, GuessGameTab::TAB_NAME);

	_mimicGameTab = new MimicGameTab;
	_tabs->addTab(_mimicGameTab, MimicGameTab::TAB_NAME);

	//Connect required slots to model signals

	//First connect phrase map load signals
	QObject::connect(model.get(), SIGNAL(generalPhraseGroupLoaded(std::map<std::string, PhraseGroupData>&)),
			_phraseTab, SLOT(onPhraseGroupLoaded(std::map<std::string, PhraseGroupData>&)));

	QObject::connect(model.get(), SIGNAL(guessGamePhraseGroupLoaded(std::map<std::string, PhraseGroupData>&)),
			_guessGameTab, SLOT(onPhraseGroupLoaded(std::map<std::string, PhraseGroupData>&)));

	QObject::connect(model.get(), SIGNAL(mimicGamePhraseGroupLoaded(std::map<std::string, PhraseGroupData>&)),
			_mimicGameTab, SLOT(onPhraseGroupLoaded(std::map<std::string, PhraseGroupData>&)));
}
