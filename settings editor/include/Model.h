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

	PhraseGroupData& getGeneralPhraseGroup(std::string key);
	const PhraseGroupData& getGeneralPhraseGroup(std::string key) const;

	PhraseGroupData& getGuessGamePhraseGroup(std::string key);
	const PhraseGroupData& getGuessGamePhraseGroup(std::string key) const;

	PhraseGroupData& getMimicGamePhraseGroup(std::string key);
	const PhraseGroupData& getMimicGamePhraseGroup(std::string key) const;

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

	void behaviorsLoaded(std::list<BehaviorData> behaviorData);
	void rewardBehaviorsLoaded(std::list<std::string> rewardBehaviors);

	void generalPhraseGroupLoaded(std::map<std::string, PhraseGroupData>&);
	void guessGamePhraseGroupLoaded(std::map<std::string, PhraseGroupData>&);
	void mimicGamePhraseGroupLoaded(std::map<std::string, PhraseGroupData>&);

};

#endif /* MODEL_H_ */
