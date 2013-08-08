#include <Controller.h>

#include <stdexcept>

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

void Controller::onRequestGameBehaviors(const std::string& name)
{
	_model->retrieveGameBehavior(name);
}
