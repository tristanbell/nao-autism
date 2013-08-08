/*
 * Model.h
 *
 *  Created on: 5 Aug 2013
 *      Author: rapid
 */

#ifndef MODEL_H_
#define MODEL_H_

#include <json/json.h>
#include <PhraseGroupData.h>
#include <BehaviorData.h>

#include <QObject>

#include <string>
#include <map>

class Model : public QObject
{

	Q_OBJECT

public:
	Model(){ }

	void retrieveGeneralPhraseGroup(const std::string& key) const;
	void retrieveGuessGamePhraseGroup(const std::string& key) const;
	void retrieveMimicGamePhraseGroup(const std::string& key) const;

	void addGeneralPhrase(std::string& key, std::string& phrase);
	void addGuessGamePhrase(const std::string& key, std::string& phrase);
	void addMimicGamePhrase(const std::string& key, std::string& phrase);

	void loadData(Json::Value& docRoot);

private:
	std::map<std::string, PhraseGroupData> _generalPhraseMap;
	std::map<std::string, PhraseGroupData> _guessGamePhraseMap;
	std::map<std::string, PhraseGroupData> _mimicGamePhraseMap;

	std::list<BehaviorData> _behaviorDataList;
	std::list<std::string> _rewardBehaviorDataList;

	void update();

signals:
	void behaviorsCleared();
	void generalPhrasesCleared();
	void guessGamePhrasesCleared();
	void mimicGamePhrasesCleared();

	void behaviorsLoaded(const std::list<BehaviorData> behaviorData);
	void rewardBehaviorsLoaded(const std::list<std::string> rewardBehaviors);

	void generalPhraseGroupLoaded(const std::map<std::string, PhraseGroupData>&);
	void guessGamePhraseGroupLoaded(const std::map<std::string, PhraseGroupData>&);
	void mimicGamePhraseGroupLoaded(const std::map<std::string, PhraseGroupData>&);

	void generalPhraseGroupRetrieved(const PhraseGroupData&) const;
	void guessGamePhraseGroupRetrieved(const PhraseGroupData&) const;
	void mimicGamePhraseGroupRetrieved(const PhraseGroupData&) const;

};

#endif /* MODEL_H_ */
