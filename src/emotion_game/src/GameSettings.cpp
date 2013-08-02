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

void GameSettings::setPhraseMap(std::map<std::string, std::list<Phrase> >& phraseMap)
{
	this->phraseMap = phraseMap;
}

const std::map<std::string, std::list<Phrase> >& GameSettings::getPhraseMap() const
{
	return phraseMap;
}

bool GameSettings::getPhraseList(const std::string& key, std::list<Phrase>& phraseList) const
{
	try{
		phraseList = phraseMap.at(key);
		return true;
	}catch(std::out_of_range& ex){
		return false;
	}
}
