/*
 * GameSettings.h
 *
 *  Created on: 29 Jul 2013
 *      Author: alex
 */

#ifndef GAMESETTINGS_H_
#define GAMESETTINGS_H_

#include <Phrase.h>

#include <map>
#include <string>

class GameSettings
{

public:
	GameSettings();

	void setWait(int msWait);
	int getWait() const;

	void setTimeout(int timeout);
	int getTimeout() const;

	void setPhraseMap(std::map<std::string, Phrase> phraseMap);
	const std::map<std::string, Phrase>& getPhraseMap() const;

private:
	int msWait;
	int timeout;
	std::map<std::string, Phrase> phraseMap;


};


#endif /* GAMESETTINGS_H_ */
