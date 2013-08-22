#include <BehaviorsTab.h>

const QString BehaviorsTab::TAB_NAME = "Behaviors";

void BehaviorsTab::init()
{
	_gameBehaviorsTab = new GameBehaviorsTab;
	_rewardBehaviorsTab = new RewardBehaviorsTab;

	addTab(_gameBehaviorsTab, GameBehaviorsTab::TAB_NAME);
	addTab(_rewardBehaviorsTab, RewardBehaviorsTab::TAB_NAME);
}

GameBehaviorsTab* BehaviorsTab::getGameBehaviorsTab()
{
	return _gameBehaviorsTab;
}

RewardBehaviorsTab* BehaviorsTab::getRewardBehaviorsTab()
{
	return _rewardBehaviorsTab;
}
