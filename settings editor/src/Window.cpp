#include <Window.h>

#include <QGridLayout>

void Window::init()
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
}
