#include <Model.h>
#include <Keys.h>

#include <fstream>
#include <stdexcept>

inline void addPhraseToPhraseGroup(std::map<std::string, PhraseGroupData>& map, const std::string& key, const std::string& phrase);
inline void addBehaviorToPhraseGroup(std::map<std::string, PhraseGroupData>& map, const std::string& key, const std::string& phrase);

inline void removePhraseFromPhraseGroup(std::map<std::string, PhraseGroupData>& map, const std::string& key, const std::string& phrase);
inline void removeBehaviorFromPhraseGroup(std::map<std::string, PhraseGroupData>& map, const std::string& key, const std::string& behavior);

void addPhraseMapToDoc(Json::Value& root, std::map<std::string, PhraseGroupData>& map);
void addBehaviorListToDoc(Json::Value& root, std::list<BehaviorData>& behaviorData);
void addStringArrayToDoc(Json::Value& root, std::list<std::string>& list);

std::map<std::string, PhraseGroupData> loadPhraseGroups(Json::Value& root);
std::list<BehaviorData> loadBehaviorData(Json::Value& root);
std::string loadBehaviorActualName(Json::Value& root);
std::list<std::string> loadBehaviorNames(Json::Value& root);
int loadClassification(Json::Value& root);

void Model::setBaseSettingsData(const BaseSettingsData& data)
{
	_settingsData = data;
}

void Model::open(const std::string& location)
{
	if (location != ""){
		//Load file into 'buffer'
		std::string jsonData;

		std::fstream ifs;
		ifs.open(location.c_str(), std::fstream::in);

		ifs.seekg(0, std::ios::end);
		jsonData.resize(ifs.tellg());
		ifs.seekg(0, std::ios::beg);

		ifs.read(&jsonData[0], jsonData.size());
		ifs.close();

		Json::Reader jsonReader;

		Json::Value doc;
		if (jsonReader.parse(jsonData, doc)){
			try{
				loadData(doc);

				_fileLocation = location;

				//Alert slots about successful open
				emit successfulOpen(location);
			}catch(MissingDataException& missingData){
				emit unsuccessfulOpen(missingData.what);
			}
		}else{
			//Alert slots about unsuccessful open
			emit unsuccessfulOpen("The file: " + location + " is not a valid JSON file.");
		}
	}
}

void Model::save()
{
	saveData(_fileLocation);
}

void Model::saveAs(const std::string& location)
{
	saveData(location);
}

void Model::saveData(const std::string& location)
{
	//Create new JSON document
	Json::Value doc(Json::objectValue);

	addPhraseMapToDoc(doc, _generalPhraseMap);
	addBehaviorListToDoc(doc, _gameBehaviorsDataList);

	//Add base settings to JSON document
	Json::Value& baseSettings = doc[BASE_SETTINGS_KEY];
	Json::Value& maxPromptVal = baseSettings[MAX_PROMPT_KEY];
	maxPromptVal = _settingsData._tries;

	Json::Value& timeoutVal = baseSettings[TIMEOUT_SETTING_KEY];
	timeoutVal = _settingsData._timeout;

	Json::Value& speechWaitVal = baseSettings[SPEECH_WAIT_SETTING_KEY];
	speechWaitVal = _settingsData._wait;

	Json::Value& confidenceVal = baseSettings[SPEECH_RECOGNITION_CONFIDENCE_KEY];
	confidenceVal = _settingsData._confidence;

	Json::Value& rewardBehavior = doc[REWARD_BEHAVIOR_LIST_KEY];
	Json::Value& rewardBehaviorNames = rewardBehavior[BEHAVIOR_NAME_KEY];
	addStringArrayToDoc(rewardBehaviorNames, _rewardBehaviorDataList);

	Json::Value& guessGameSettings = doc[GUESS_GAME_KEY];
	addPhraseMapToDoc(guessGameSettings, _guessGamePhraseMap);

	Json::Value& mimicGameSettings = doc[MIMIC_GAME_KEY];
	addPhraseMapToDoc(mimicGameSettings, _mimicGamePhraseMap);

	//Generate styled json string
	Json::StyledWriter writer;
	std::string output = writer.write(doc);

	//Write generated json string to file and flush
	std::fstream fs;
	fs.open(location.c_str(), std::fstream::out);

	fs << output;
	fs.flush();

	//Close stream
	fs.close();

	//Alert slots about successful save
	emit successfulSave(location);
}

void addPhraseMapToDoc(Json::Value& root, std::map<std::string, PhraseGroupData>& map){
	Json::Value& phraseRoot = root[PHRASE_KEY];

	std::map<std::string, PhraseGroupData>::iterator it = map.begin();
	while (it != map.end()){
		std::pair<std::string, PhraseGroupData> pair = *it;

		Json::Value& current = phraseRoot[pair.first];

		//Now, add phrases and behaviors
		Json::Value& currentPhrases = current[PHRASE_KEY];
		std::list<std::string>::iterator phraseIt = pair.second.phraseVector.begin();
		while (phraseIt != pair.second.phraseVector.end()){
			std::string& phrase = *phraseIt;

			currentPhrases.append(phrase);
			phraseIt++;
		}

		Json::Value& currentBehaviors = current[BEHAVIOR_KEY];
		std::list<std::string>::iterator behaviorIt = pair.second.behaviorVector.begin();
		while (behaviorIt != pair.second.behaviorVector.end()){
			std::string& behavior = *behaviorIt;

			currentBehaviors.append(behavior);
			behaviorIt++;
		}

		it++;
	}
}

void addBehaviorListToDoc(Json::Value& root, std::list<BehaviorData>& behaviorData)
{
	Json::Value& behaviorRoot = root[BEHAVIOR_KEY];

	std::list<BehaviorData>::iterator it = behaviorData.begin();
	while (it != behaviorData.end()){
		BehaviorData& data = *it;

		Json::Value newBehavior(Json::objectValue);

		Json::Value& newBehaviorActualName = newBehavior[BEHAVIOR_ACTUAL_KEY];
		newBehaviorActualName = data._actualName;

		Json::Value& newBehaviorNames = newBehavior[BEHAVIOR_NAME_KEY];
		addStringArrayToDoc(newBehaviorNames, data._behaviorNames);

		Json::Value& newBehaviorClassification = newBehavior[BEHAVIOR_CLASSIFICATION_KEY];
		newBehaviorClassification = data._classification;

		behaviorRoot.append(newBehavior);

		it++;
	}
}

void addStringArrayToDoc(Json::Value& root, std::list<std::string>& list)
{
	std::list<std::string>::iterator it = list.begin();
	while (it != list.end()){
		std::string& str = *it;

		root.append(str);

		it++;
	}
}

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

void Model::addGeneralPhrase(const std::string& key, const std::string& phrase)
{
	addPhraseToPhraseGroup(_generalPhraseMap, key, phrase);
}

void Model::addGuessGamePhrase(const std::string& key, std::string& phrase)
{
	addPhraseToPhraseGroup(_guessGamePhraseMap, key, phrase);
}

void Model::addMimicGamePhrase(const std::string& key, std::string& phrase)
{
	addPhraseToPhraseGroup(_mimicGamePhraseMap, key, phrase);
}

inline void addPhraseToPhraseGroup(std::map<std::string, PhraseGroupData>& map, const std::string& key, const std::string& phrase)
{
	try{
		PhraseGroupData& data = map.at(key);

		data.phraseVector.push_back(phrase);
	}catch(std::out_of_range& ex){  }
}

void Model::removeGeneralPhrase(const std::string& key, const std::string& phrase)
{
	removePhraseFromPhraseGroup(_generalPhraseMap, key, phrase);
}

void Model::removeGuessGamePhrase(const std::string& key, const std::string& phrase)
{
	removePhraseFromPhraseGroup(_guessGamePhraseMap, key, phrase);
}

void Model::removeMimicGamePhrase(const std::string& key, const std::string& phrase)
{
	removePhraseFromPhraseGroup(_mimicGamePhraseMap, key, phrase);
}

void removePhraseFromPhraseGroup(std::map<std::string, PhraseGroupData>& map, const std::string& key, const std::string& phrase)
{
	try{
		PhraseGroupData& data = map.at(key);

		data.phraseVector.remove(phrase);
	}catch (std::out_of_range& ex){  }
}

void Model::addGeneralPhraseBehavior(const std::string& key, const std::string& behavior)
{
	addBehaviorToPhraseGroup(_generalPhraseMap, key, behavior);
}

void Model::addGuessGamePhraseBehavior(const std::string& key, const std::string& behavior)
{
	addBehaviorToPhraseGroup(_guessGamePhraseMap, key, behavior);
}

void Model::addMimicGamePhraseBehavior(const std::string& key, const std::string& behavior)
{
	addBehaviorToPhraseGroup(_mimicGamePhraseMap, key, behavior);
}

inline void addBehaviorToPhraseGroup(std::map<std::string, PhraseGroupData>& map, const std::string& key, const std::string& behavior)
{
	try{
		PhraseGroupData& data = map.at(key);

		data.behaviorVector.push_back(behavior);
	}catch(std::out_of_range& ex){  }
}

void Model::removeGeneralPhraseBehavior(const std::string& key, const std::string& behavior)
{
	removeBehaviorFromPhraseGroup(_generalPhraseMap, key, behavior);
}

void Model::removeGuessGamePhraseBehavior(const std::string& key, const std::string& behavior)
{
	removeBehaviorFromPhraseGroup(_guessGamePhraseMap, key, behavior);
}

void Model::removeMimicGamePhraseBehavior(const std::string& key, const std::string& behavior)
{
	removeBehaviorFromPhraseGroup(_mimicGamePhraseMap, key, behavior);
}

void removeBehaviorFromPhraseGroup(std::map<std::string, PhraseGroupData>& map, const std::string& key, const std::string& behavior)
{
	try{
		PhraseGroupData& data = map.at(key);

		data.behaviorVector.remove(behavior);
	}catch (std::out_of_range& ex){  }
}

void Model::addGameBehavior(const std::string& actualName, const std::string& behaviorName)
{
	std::list<BehaviorData>::iterator it = _gameBehaviorsDataList.begin();
	while (it != _gameBehaviorsDataList.end()){
		BehaviorData& data = *it;

		if (data._actualName == actualName){
			data._behaviorNames.push_back(behaviorName);
		}

		it++;
	}
}

void Model::removeGameBehavior(const std::string& actualName, const std::string& behaviorName)
{
	std::cout << "Removing...\n";
	std::list<BehaviorData>::iterator it = _gameBehaviorsDataList.begin();
	while (it != _gameBehaviorsDataList.end()){
		BehaviorData& data = *it;

		if (data._actualName == actualName){
			data._behaviorNames.remove(behaviorName);
		}

		it++;
	}
}

void Model::retrieveGameBehavior(const std::string& name) const
{
	std::list<BehaviorData>::const_iterator it = _gameBehaviorsDataList.begin();
	while (it != _gameBehaviorsDataList.end()){
		const BehaviorData& data = *it;

		if (data._actualName == name){
			emit gameBehaviorRetrieved(data);
			return;
		}

		it++;
	}
}

void Model::addRewardBehavior(const std::string& name)
{
	_rewardBehaviorDataList.push_back(name);
}

void Model::removeRewardBehavior(const std::string& name)
{
	_rewardBehaviorDataList.remove(name);
}

void Model::loadData(Json::Value& docRoot){
	//Load base settings
	Json::Value& baseSettings = docRoot[BASE_SETTINGS_KEY];
	if (baseSettings != Json::Value::null){
		Json::Value& timeoutVal = baseSettings[TIMEOUT_SETTING_KEY];
		_settingsData._timeout = timeoutVal.asInt();

		Json::Value& maxPromptVal = baseSettings[MAX_PROMPT_KEY];
		_settingsData._tries = maxPromptVal.asInt();

		Json::Value& waitVal = baseSettings[SPEECH_WAIT_SETTING_KEY];
		_settingsData._wait = waitVal.asInt();

		Json::Value& confidenceVal = baseSettings[SPEECH_RECOGNITION_CONFIDENCE_KEY];
		_settingsData._confidence = confidenceVal.asFloat();
	}else{
		MissingDataException missing;
		missing.what = "Missing base settings.";

		throw missing;
	}

	//Load general phrase map
	Json::Value& generalPhrases = docRoot[PHRASE_KEY];
	if (generalPhrases != Json::Value::null){
		//Found general phrases, load them into phrase groups
		_generalPhraseMap = loadPhraseGroups(generalPhrases);
	}else{
		MissingDataException missing;
		missing.what = "Missing general phrases.";

		throw missing;
	}

	//Load guess game phrase map
	Json::Value& guessGamePhrases = docRoot[GUESS_GAME_KEY][PHRASE_KEY];
	if (guessGamePhrases != Json::Value::null){
		_guessGamePhraseMap = loadPhraseGroups(guessGamePhrases);
	}else{
		MissingDataException missing;
		missing.what = "Missing guessing game settings.";

		throw missing;
	}

	//Load mimic game phrase map
	Json::Value& mimicGamePhrases = docRoot[MIMIC_GAME_KEY][PHRASE_KEY];
	if (mimicGamePhrases != Json::Value::null){
		_mimicGamePhraseMap = loadPhraseGroups(mimicGamePhrases);
	}else{
		MissingDataException missing;
		missing.what = "Missing mimic game settings.";

		throw missing;
	}

	//Load reward behaviors
	Json::Value& rewardBehaviors = docRoot[REWARD_BEHAVIOR_LIST_KEY][BEHAVIOR_NAME_KEY];
	if (rewardBehaviors != Json::Value::null){
		_rewardBehaviorDataList = loadBehaviorNames(rewardBehaviors);
	}else{
		MissingDataException missing;
		missing.what = "Missing reward behaviors.";

		throw missing;
	}

	//Load other behaviors
	Json::Value& otherBehaviors = docRoot[BEHAVIOR_KEY];
	if (otherBehaviors != Json::Value::null){
		_gameBehaviorsDataList = loadBehaviorData(otherBehaviors);
	}else{
		MissingDataException missing;
		missing.what = "Missing general behaviors.";

		throw missing;
	}

	//Fire all required signals for the new data
	update();
}

void Model::update()
{
	emit baseSettingsLoaded(_settingsData);

	emit generalPhraseGroupLoaded(_generalPhraseMap);
	emit guessGamePhraseGroupLoaded(_guessGamePhraseMap);
	emit mimicGamePhraseGroupLoaded(_mimicGamePhraseMap);

	emit gameBehaviorsLoaded(_gameBehaviorsDataList);
	emit rewardBehaviorsLoaded(_rewardBehaviorDataList);
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
