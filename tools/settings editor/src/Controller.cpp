#include <Controller.h>

#include <stdexcept>

void Controller::onBaseSettingsUpdated(const BaseSettingsData& data)
{
	_model->setBaseSettingsData(data);
}

void Controller::onOpenRequested(const std::string& location)
{
	_model->open(location);
}

void Controller::onSaveRequested()
{
	_model->save();
}

void Controller::onSaveAsRequested(const std::string& location)
{
	_model->saveAs(location);
}

void Controller::onRequestGeneralPhraseGroup(const std::string& key)
{
	_model->retrieveGeneralPhraseGroup(key);
}

void Controller::onRequestGuessGamePhraseGroup(const std::string& key)
{
	_model->retrieveGuessGamePhraseGroup(key);
}

void Controller::onRequestMimicGamePhraseGroup(const std::string& key)
{
	_model->retrieveMimicGamePhraseGroup(key);
}

void Controller::onGeneralPhraseCreated(std::string& key, std::string& phrase)
{
	_model->addGeneralPhrase(key, phrase);
}

void Controller::onGuessGamePhraseCreated(std::string& key, std::string& phrase)
{
	_model->addGuessGamePhrase(key, phrase);
}

void Controller::onMimicGamePhraseCreated(std::string& key, std::string& phrase)
{
	_model->addMimicGamePhrase(key, phrase);
}

void Controller::onGeneralPhraseRemoved(const std::string& key, const std::string& phrase)
{
	_model->removeGeneralPhrase(key, phrase);
}

void Controller::onGuessGamePhraseRemoved(const std::string& key, const std::string& phrase)
{
	_model->removeGuessGamePhrase(key, phrase);
}

void Controller::onMimicGamePhraseRemoved(const std::string& key, const std::string& phrase)
{
	_model->removeMimicGamePhrase(key, phrase);
}

void Controller::onGeneralPhraseBehaviorCreated(std::string& key, std::string& behavior)
{
	_model->addGeneralPhraseBehavior(key, behavior);
}

void Controller::onGuessGamePhraseBehaviorCreated(std::string& key, std::string& behavior)
{
	_model->addGuessGamePhraseBehavior(key, behavior);
}

void Controller::onMimicGamePhraseBehaviorCreated(std::string& key, std::string& behavior)
{
	_model->addMimicGamePhraseBehavior(key, behavior);
}

void Controller::onGeneralPhraseBehaviorRemoved(const std::string& key, const std::string& behavior)
{
	_model->removeGeneralPhraseBehavior(key, behavior);
}

void Controller::onGuessGamePhraseBehaviorRemoved(const std::string& key, const std::string& behavior)
{
	_model->removeGuessGamePhraseBehavior(key, behavior);
}

void Controller::onMimicGamePhraseBehaviorRemoved(const std::string& key, const std::string& behavior)
{
	_model->removeMimicGamePhraseBehavior(key, behavior);
}

void Controller::onGameBehaviorCreated(const std::string& actualName, const std::string& behaviorName)
{
	_model->addGameBehavior(actualName, behaviorName);
}

void Controller::onGameBehaviorRemoved(const std::string& actualName, const std::string& behaviorName)
{
	_model->removeGameBehavior(actualName, behaviorName);
}

void Controller::onRequestGameBehaviors(const std::string& name)
{
	_model->retrieveGameBehavior(name);
}

void Controller::onAddRewardBehavior(const std::string& name)
{
	_model->addRewardBehavior(name);
}

void Controller::onRemoveRewardBehavior(const std::string& name)
{
	_model->removeRewardBehavior(name);
}

void Controller::loadNewData()
{
	_model->defaultData();
}
