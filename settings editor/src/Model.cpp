#include <Model.h>
#include <Keys.h>

#include <stdexcept>

std::map<std::string, PhraseGroupData> loadPhraseGroups(Json::Value& root);
std::list<BehaviorData> loadBehaviorData(Json::Value& root);
std::string loadBehaviorActualName(Json::Value& root);
std::list<std::string> loadBehaviorNames(Json::Value& root);
int loadClassification(Json::Value& root);

void Model::retrieveGeneralPhraseGroup(const std::string& key) const
{
	try{
		const PhraseGroupData& data = _generalPhraseMap.at(key);

		emit generalPhraseGroupRetrieved(data);
	}catch(std::out_of_range& ex){  }
}

void Model::retrieveGuessGamePhraseGroup(const std::string& key) const
{
	try{
		const PhraseGroupData& data = _guessGamePhraseMap.at(key);

		emit guessGamePhraseGroupRetrieved(data);
	}catch(std::out_of_range& ex){  }
}

void Model::retrieveMimicGamePhraseGroup(const std::string& key) const
{
	try{
		const PhraseGroupData& data = _mimicGamePhraseMap.at(key);

		emit mimicGamePhraseGroupRetrieved(data);
	}catch(std::out_of_range& ex){  }
}

void Model::loadData(Json::Value& docRoot){
	//Load general phrase map
	Json::Value& generalPhrases = docRoot[PHRASE_KEY];
	if (generalPhrases != Json::Value::null){
		//Found general phrases, load them into phrase groups
		_generalPhraseMap = loadPhraseGroups(generalPhrases);
	}

	//Load guess game phrase map
	Json::Value& guessGamePhrases = docRoot[GUESS_GAME_KEY][PHRASE_KEY];
	if (guessGamePhrases != Json::Value::null){
		_guessGamePhraseMap = loadPhraseGroups(guessGamePhrases);
	}

	//Load mimic game phrase map
	Json::Value& mimicGamePhrases = docRoot[MIMIC_GAME_KEY][PHRASE_KEY];
	if (mimicGamePhrases != Json::Value::null){
		_mimicGamePhraseMap = loadPhraseGroups(mimicGamePhrases);
	}

	//Load reward behaviors
	Json::Value& rewardBehaviors = docRoot[REWARD_BEHAVIOR_LIST_KEY][BEHAVIOR_NAME_KEY];
	if (rewardBehaviors != Json::Value::null){
		_rewardBehaviorDataList = loadBehaviorNames(rewardBehaviors);
	}

	//Load other behaviors
	Json::Value& otherBehaviors = docRoot[BEHAVIOR_KEY];
	if (otherBehaviors != Json::Value::null){
		_behaviorDataList = loadBehaviorData(otherBehaviors);
	}

	//Fire all required signals for the new data
	update();
}

void Model::update()
{
	emit generalPhraseGroupLoaded(_generalPhraseMap);
	emit guessGamePhraseGroupLoaded(_guessGamePhraseMap);
	emit mimicGamePhraseGroupLoaded(_mimicGamePhraseMap);
}

std::map<std::string, PhraseGroupData> loadPhraseGroups(Json::Value& root){
	std::map<std::string, PhraseGroupData> loaded;

	Json::Value::Members memberNames = root.getMemberNames();
	for (int i=0;i<memberNames.size();i++){
		std::string key = memberNames[i];

		Json::Value current = root.get(key, Json::Value::null);
		if (current != Json::Value::null){
			std::list<std::string> phrases;
			std::list<std::string> behaviors;

			Json::Value phraseArray = current.get(PHRASE_KEY, Json::Value::null);
			if (phraseArray != Json::Value::null){
				Json::Value::ArrayIndex size = phraseArray.size();

				for (int j=0;j<size;j++){
					Json::Value phraseNameVal = phraseArray.get(j, Json::Value::null);

					if (phraseNameVal != Json::Value::null){
						phrases.push_back(phraseNameVal.asString());
					}
				}
			}

			Json::Value behaviorArray = current.get(BEHAVIOR_KEY, Json::Value::null);
			if (behaviorArray != Json::Value::null){
				Json::Value::ArrayIndex size = behaviorArray.size();

				for (int j=0;j<size;j++){
					Json::Value behaviorNameVal = behaviorArray.get(j, Json::Value::null);

					if (behaviorNameVal != Json::Value::null){
						behaviors.push_back(behaviorNameVal.asString());
					}
				}
			}

			PhraseGroupData group;

			group.phraseVector = phrases;
			group.behaviorVector = behaviors;

			loaded.insert(std::pair<std::string, PhraseGroupData>(key, group));
		}
	}

	return loaded;
}

std::list<BehaviorData> loadBehaviorData(Json::Value& root){
	std::list<BehaviorData> loaded;

	Json::Value::ArrayIndex size = root.size();
	for (int i=0;i<size;i++){
		Json::Value& val = root[i];

		//Sanity check
		if (val != Json::Value::null){
			Json::Value& actualVal = val[BEHAVIOR_ACTUAL_KEY];
			std::string actual = loadBehaviorActualName(actualVal);

			Json::Value& namesVal = val[BEHAVIOR_NAME_KEY];
			std::list<std::string> names = loadBehaviorNames(namesVal);

			Json::Value& classificationVal = val[BEHAVIOR_CLASSIFICATION_KEY];
			int classification = loadClassification(classificationVal);

			BehaviorData data;
			data._actualName = actual;
			data._behaviorNames = names;
			data._classification = classification;

			loaded.push_back(data);
		}
	}

	return loaded;
}

std::string loadBehaviorActualName(Json::Value& root){
	if (root != Json::Value::null){
		return root.asString();
	}

	return "";
}

std::list<std::string> loadBehaviorNames(Json::Value& root){
	std::list<std::string> loaded;

	if (root != Json::Value::null){
		Json::Value::ArrayIndex size = root.size();
		for (int i=0;i<size;i++){
			Json::Value& current = root[i];

			if (current != Json::Value::null){
				loaded.push_back(current.asString());
			}
		}
	}

	return loaded;
}

int loadClassification(Json::Value& root){
	if (root != Json::Value::null){
		return root.asInt();
	}

	return -1;
}
