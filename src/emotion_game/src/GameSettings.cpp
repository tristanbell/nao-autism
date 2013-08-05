#include <GameSettings.h>

#include <stdexcept>

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

void GameSettings::setMaxPromptAmount(int amount)
{
	this->defaultPrompts = amount;
}

int GameSettings::getMaxPromptAmount() const
{
	return defaultPrompts;
}

void GameSettings::setBehaviorVector(std::vector<Behavior>& behaviorVector)
{
	this->behaviorVector = behaviorVector;
}

const std::vector<Behavior>& GameSettings::getBehaviorVector() const
{
	return behaviorVector;
}

void GameSettings::setPhraseMap(std::map<std::string, std::vector<Phrase> >& phraseMap)
{
	this->phraseMap = phraseMap;
}

const std::map<std::string, std::vector<Phrase> >& GameSettings::getPhraseMap() const
{
	return phraseMap;
}

bool GameSettings::getPhraseVector(const std::string& key, std::vector<Phrase>& phraseVector) const
{
	try{
		phraseVector = phraseMap.at(key);
		return true;
	}catch(std::out_of_range& ex){
		return false;
	}

	return false;
}
