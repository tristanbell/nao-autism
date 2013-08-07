#include <Controller.h>

#include <stdexcept>

void Controller::onRequestGeneralPhraseGroup(const std::string& key)
{
	_modelPtr.get()->retrieveGeneralPhraseGroup(key);
}

void Controller::onRequestGuessGamePhraseGroup(const std::string& key)
{
	_modelPtr.get()->retrieveGuessGamePhraseGroup(key);
}

void Controller::onRequestMimicGamePhraseGroup(const std::string& key)
{
	_modelPtr.get()->retrieveMimicGamePhraseGroup(key);
}
