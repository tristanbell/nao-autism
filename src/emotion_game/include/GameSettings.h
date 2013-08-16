/*
 * GameSettings.h
 *
 *  Created on: 29 Jul 2013
 *      Author: alex
 */

#ifndef GAMESETTINGS_H_
#define GAMESETTINGS_H_

#include <Phrase.h>
#include <Behavior.h>

#include <map>
#include <vector>
#include <string>

class GameSettings
{

public:
	GameSettings();

	void setWait(int msWait);
	int getWait() const;

	void setTimeout(int timeout);
	int getTimeout() const;

	void setMaxPromptAmount(int amount);
	int getMaxPromptAmount() const;

	void setBehaviorVector(std::vector<Behavior>& behaviorVector);
	const std::vector<Behavior>& getBehaviorVector() const;

	void setPhraseMap(std::map<std::string, std::vector<Phrase> >& phraseMap);
	const std::map<std::string, std::vector<Phrase> >& getPhraseMap() const;
	bool getPhraseVector(const std::string& key, std::vector<Phrase>& phraseList) const;

	void setConfidenceThreshold(float confidenceThreshold);
	float getConfidenceThreshold() const;

private:
	int msWait;
	int timeout;
	int defaultPrompts;
	float confidenceThreshold;

	std::vector<Behavior> behaviorVector;
	std::map<std::string, std::vector<Phrase> > phraseMap;

};


#endif /* GAMESETTINGS_H_ */
