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

	void open(const std::string& location);

	void save();
	void saveAs(const std::string& location);

	void retrieveGeneralPhraseGroup(const std::string& key) const;
	void retrieveGuessGamePhraseGroup(const std::string& key) const;
	void retrieveMimicGamePhraseGroup(const std::string& key) const;

	void addGeneralPhrase(const std::string& key, const std::string& phrase);
	void addGuessGamePhrase(const std::string& key, std::string& phrase);
	void addMimicGamePhrase(const std::string& key, std::string& phrase);

	void addGeneralPhraseBehavior(const std::string& key, const std::string& behavior);
	void addGuessGamePhraseBehavior(const std::string& key, const std::string& behavior);
	void addMimicGamePhraseBehavior(const std::string& key, const std::string& behavior);

	void retrieveGameBehavior(const std::string& name) const;

	void loadData(Json::Value& docRoot);

private:
	std::string _fileLocation;

	std::map<std::string, PhraseGroupData> _generalPhraseMap;
	std::map<std::string, PhraseGroupData> _guessGamePhraseMap;
	std::map<std::string, PhraseGroupData> _mimicGamePhraseMap;

	std::list<BehaviorData> _gameBehaviorsDataList;
	std::list<std::string> _rewardBehaviorDataList;

	void update();

signals:
	void behaviorsCleared();
	void generalPhrasesCleared();
	void guessGamePhrasesCleared();
	void mimicGamePhrasesCleared();

	void gameBehaviorsLoaded(const std::list<BehaviorData>& behaviorData);
	void gameBehaviorRetrieved(const BehaviorData&) const;

	void rewardBehaviorsLoaded(const std::list<std::string>& rewardBehaviors);

	void generalPhraseGroupLoaded(const std::map<std::string, PhraseGroupData>&);
	void guessGamePhraseGroupLoaded(const std::map<std::string, PhraseGroupData>&);
	void mimicGamePhraseGroupLoaded(const std::map<std::string, PhraseGroupData>&);

	void generalPhraseGroupRetrieved(const PhraseGroupData&) const;
	void guessGamePhraseGroupRetrieved(const PhraseGroupData&) const;
	void mimicGamePhraseGroupRetrieved(const PhraseGroupData&) const;

};

#endif /* MODEL_H_ */
