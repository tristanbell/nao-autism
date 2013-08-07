#include <Controller.h>

#include <stdexcept>

PhraseGroupData& Controller::getGeneralPhraseGroup(std::string key)
{
	return _modelPtr.get()->getGeneralPhraseGroup(key);
}

const PhraseGroupData& Controller::getGeneralPhraseGroup(std::string key) const
{
	return _modelPtr.get()->getGeneralPhraseGroup(key);
}

PhraseGroupData& Controller::getGuessGamePhraseGroup(std::string key)
{
	return _modelPtr.get()->getGuessGamePhraseGroup(key);
}

const PhraseGroupData& Controller::getGuessGamePhraseGroup(std::string key) const
{
	return _modelPtr.get()->getGuessGamePhraseGroup(key);
}

PhraseGroupData& Controller::getMimicGamePhraseGroup(std::string key)
{
	return _modelPtr.get()->getMimicGamePhraseGroup(key);
}

const PhraseGroupData& Controller::getMimicGamePhraseGroup(std::string key) const
{
	return _modelPtr.get()->getMimicGamePhraseGroup(key);
}
