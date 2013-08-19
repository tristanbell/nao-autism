#include <BehaviorsTab.h>

const QString BehaviorsTab::TAB_NAME = "Behaviors";

void BehaviorsTab::init()
{
	_gameBehaviorsTab = new GameBehaviorsTab;

	addTab(_gameBehaviorsTab, GameBehaviorsTab::TAB_NAME);
}

GameBehaviorsTab* BehaviorsTab::getGameBehaviorsTab()
{
	return _gameBehaviorsTab;
}
