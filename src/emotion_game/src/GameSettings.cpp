#include <GameSettings.h>

GameSettings::GameSettings(){ }

void GameSettings::setWait(int msWait)
{
	this->msWait = msWait;
}

int GameSettings::getWait() const
{
	return msWait;
}

void GameSettings::setTimeout(int timeout)
{
	this->timeout = timeout;
}

int GameSettings::getTimeout() const
{
	return timeout;
}

void GameSettings::setPhraseMap(std::map<std::string, Phrase> phraseMap)
{
	this->phraseMap = phraseMap;
}

const std::map<std::string, Phrase>& GameSettings::getPhraseMap() const
{
	return phraseMap;
}
