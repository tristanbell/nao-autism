#include <Model.h>

#include <Keys.h>

std::map<QString, PhraseGroupData> loadPhraseGroups(Json::Value& root);

void Model::setData(Json::Value& doc){
	docRoot = doc;

	update();
}

void Model::update()
{
	//Load general phrase group
	std::map<QString, PhraseGroupData> generalPhraseGroup;

	Json::Value generalPhrases = docRoot.get(PHRASE_KEY, Json::Value::null);
	if (generalPhrases != Json::Value::null){
		//Found general phrases, load them into phrase groups
		generalPhraseGroup = loadPhraseGroups(generalPhrases);
	}

	emit generalPhraseGroupLoaded(generalPhraseGroup);
}

std::map<QString, PhraseGroupData> loadPhraseGroups(Json::Value& root){
	std::map<QString, PhraseGroupData> loaded;

	Json::Value::Members memberNames = root.getMemberNames();
	for (int i=0;i<memberNames.size();i++){
		std::string key = memberNames[i];

		Json::Value current = root.get(key, Json::Value::null);
		if (current != Json::Value::null){
			std::list<QString> phrases;
			std::list<QString> behaviors;

			Json::Value phraseArray = root.get(PHRASE_KEY, Json::Value::null);
			if (phraseArray != Json::Value::null){
				Json::Value::ArrayIndex size = phraseArray.size();

				for (int j=0;j<size;j++){
					Json::Value phraseNameVal = phraseArray.get(j, Json::Value::null);

					if (phraseNameVal != Json::Value::null){
						QString name = QString::fromStdString(phraseNameVal.asString());

						phrases.push_back(name);
					}
				}
			}

			Json::Value behaviorArray = root.get(BEHAVIOR_KEY, Json::Value::null);
			if (behaviorArray != Json::Value::null){
				Json::Value::ArrayIndex size = behaviorArray.size();

				for (int j=0;j<size;j++){
					Json::Value behaviorNameVal = behaviorArray.get(j, Json::Value::null);

					if (behaviorNameVal != Json::Value::null){
						QString name = QString::fromStdString(behaviorNameVal.asString());

						behaviors.push_back(name);
					}
				}
			}

			QString qKey = QString::fromStdString(key);

			PhraseGroupData group;

			group.phraseVector = phrases;
			group.behaviorVector = behaviors;

			loaded.insert(std::pair<QString, PhraseGroupData>(qKey, group));
		}
	}

	return loaded;
}
