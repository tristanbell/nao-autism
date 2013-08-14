/*
 * main.cpp
 *
 *  Created on: 29 Jul 2013
 *      Author: tristan
 */

#include <ros/ros.h>
#include <std_srvs/Empty.h>

#include <tf/tfMessage.h>
#include <geometry_msgs/TransformStamped.h>

#include <Game.h>
#include <GuessGame.h>
#include <MimicGame.h>
#include <GameSettings.h>
#include <Behavior.h>
#include <Phrase.h>
#include <Keys.h>

#include <XmlRpcValue.h>

#include <iostream>
#include <fstream>
#include <json/json.h>

#include <string>
#include <vector>
#include <map>
#include <list>

#include <sstream>

#define VOCABULARY_KEY "nao_speech/vocabulary"

#define NODE_NAME "emotion_game"
#define OPENNI_TRACKER "/openni_tracker"

#define HEAD_TF "head_"

bool tfReady;

void initSpeechRecognition(std::vector<std::string>& vocabVector);
std::string generateXMLRPCArray(std::vector<std::string> vector);

bool checkRunningNodes(std::vector<std::string>&);
void checkTfTransforms();
void tfCallback(const tf::tfMessage);

std::vector<Behavior> getBehaviorList(Json::Value& behaviorRoot);

std::map<std::string, std::vector<Phrase> > getPhraseMap(Json::Value& phraseRoot);
bool loadPhraseMaps(Json::Value& doc, std::map<std::string, std::vector<Phrase> >& genericPhraseMap,
		std::map<std::string, std::vector<Phrase> >& guessGamePhraseMap,
		std::map<std::string, std::vector<Phrase> >& mimicGamePhraseMap);

void runGameLoop(Game* guessGame, Game* mimicGame, Game* currentGame);

int main(int argc, char** argv)
{
	if (argc != 2){
		std::cout << "Invalid arguments. The arguments should be as follows:\n"
				<< "\t<file>\n"
				<< "Where <file> is the location of the JSON data file.\n";

		return -1;
	}

	//Initialise random seed
	srand (time(NULL));

	ros::init(argc, argv, NODE_NAME);

	std::vector<std::string> node_names;
	ros::master::getNodes(node_names);

	//Construct settings object
	GameSettings settings;

	settings.setWait(2);
	settings.setTimeout(5);

	//Read in json file
	char* fileName = argv[1];

	std::string jsonData;

	std::fstream ifs;
	ifs.open(fileName, std::fstream::in);

	ifs.seekg(0, std::ios::end);
	jsonData.resize(ifs.tellg());
	ifs.seekg(0, std::ios::beg);

	ifs.read(&jsonData[0], jsonData.size());
	ifs.close();

	Json::Reader jsonReader;

	Json::Value doc;

	if (jsonReader.parse(jsonData, doc)){
		std::cout << "Parsed json data file, extracting data." << std::endl;

		Json::Value nullValue(Json::nullValue);

		//All the required setting objects
		int wait = 3;
		int timeout = 5;
		int maxPrompts = 2;

		std::vector<Behavior> allBehaviorList;
		std::vector<Behavior> rewardBehaviorList;

		std::map<std::string, std::vector<Phrase> > genericPhraseMap, guessGamePhraseMap, mimicGamePhraseMap;
		if (!loadPhraseMaps(doc, genericPhraseMap, guessGamePhraseMap, mimicGamePhraseMap))
			return 1;

		//Load speech/timeout wait variables
		Json::Value baseSettings = doc.get(BASE_SETTINGS_KEY, nullValue);
		if (baseSettings.type() != nullValue.type()){
			Json::Value waitVal = baseSettings.get(SPEECH_WAIT_SETTING_KEY, nullValue);
			if (waitVal.type() != nullValue.type()){
				wait = waitVal.asInt();
			}

			Json::Value timeoutVal = baseSettings.get(TIMEOUT_SETTING_KEY, nullValue);
			if (timeoutVal.type() != nullValue.type()){
				timeout = timeoutVal.asInt();
			}

			Json::Value maxPromptVal = baseSettings.get(MAX_PROMPT_KEY, Json::Value::null);
			if (maxPromptVal.type() != Json::Value::null.type()){
				maxPrompts = maxPromptVal.asInt();
			}
		}else{
			ROS_ERROR("Unable to find the base settings, perhaps the json data file is invalid, run the gen_json node to generate a new json file.");

			return 1;
		}

		//Load behavior list
		Json::Value allBehaviorVal = doc.get(BEHAVIOR_KEY, Json::Value::null);
		if (allBehaviorVal.type() != Json::Value::null.type()){
			allBehaviorList = getBehaviorList(allBehaviorVal);
		}else{
			ROS_ERROR("Unable to find behaviors, perhaps the json data file is invalid, run the gen_json node to generate a new json file.");
			return 1;
		}

		//Load reward behavior list
		Json::Value rewardBehaviorVal = doc.get(REWARD_BEHAVIOR_LIST_KEY, Json::Value::null);
		if (rewardBehaviorVal.type() != Json::Value::null.type()){
			Json::Value rewardBehaviorListVal = rewardBehaviorVal[BEHAVIOR_NAME_KEY];

			Json::Value::ArrayIndex size = rewardBehaviorListVal.size();
			for (int i=0;i<size;i++){
				Json::Value current = rewardBehaviorListVal.get(i, Json::Value::null);

				if (current != Json::Value::null){
					rewardBehaviorList.push_back(current.asString());
				}
			}
		}else{
			ROS_ERROR("Unable to find behaviors, perhaps the json data file is invalid, run the gen_json node to generate a new json file.");
			return 1;
		}

		ROS_INFO("JSON data loaded, creating game objects.");

		//Create vocab vector
		std::vector<std::string> vocabularyVector;
		vocabularyVector.push_back("yes");
		vocabularyVector.push_back("no");

		for (int i=0;i<allBehaviorList.size();i++){
			Behavior& current = allBehaviorList[i];

			vocabularyVector.push_back(current.getActualName());
		}

		//Now everything is loaded, perform pre-game tests
		initSpeechRecognition(vocabularyVector);

		//Create settings and instances of games
		GameSettings guessGameSettings;

		guessGameSettings.setWait(wait);
		guessGameSettings.setTimeout(timeout);
		guessGameSettings.setMaxPromptAmount(maxPrompts);
		guessGameSettings.setBehaviorVector(allBehaviorList);
		guessGameSettings.setPhraseMap(guessGamePhraseMap);
		guessGameSettings.setConfidenceThreshold(0.25);

		GameSettings mimicGameSettings;

		mimicGameSettings.setWait(wait);
		mimicGameSettings.setTimeout(timeout);
		mimicGameSettings.setBehaviorVector(allBehaviorList);
		mimicGameSettings.setMaxPromptAmount(maxPrompts);
		mimicGameSettings.setPhraseMap(mimicGamePhraseMap);
		mimicGameSettings.setConfidenceThreshold(0.25);

		Game* guessGame = new GuessGame(guessGameSettings);
		Game* mimicGame = new MimicGame(mimicGameSettings);

		//Set current game and start it.
		Game* currentGame = guessGame;
//		Game* currentGame = mimicGame;
		currentGame->startGame();

		ROS_INFO("Game initialisation done, starting.");

		runGameLoop(guessGame, mimicGame, currentGame);
	}else{
		std::cout << "Invalid json data file." << std::endl;

		return 1;
	}


	//For now, hardcode the Phrases
	//std::map<std::string, Phrase> phraseMap;

//	while (!checkRunningNodes(node_names))
//		sleep(1);
//
//	//Check if the tf transform node is active and publishing
//	checkTfTransforms();

//	startSpeechRecognition();

	return 0;
}

void runGameLoop(Game* guessGame, Game* mimicGame, Game* currentGame)
{
	ros::Rate loopRate(40);
	//All checks are done, start game loop
	while (ros::ok()){
		if (!currentGame->isDone){
			currentGame->perform();
		}else{
			//Clean up state of current game
			currentGame->endGame();

			//Swap games
			if (currentGame == guessGame){
				currentGame = mimicGame;
			}else{
				currentGame = guessGame;
			}

			//Start the new game
			currentGame->startGame();
		}

		//Spin once to enable call backs, etc.
		ros::spinOnce();
		loopRate.sleep();
	}
}

void initSpeechRecognition(std::vector<std::string>& vocabVector)
{
	ros::NodeHandle nh;

	std::cout << "Sending parameters for speech recognition.\n";

	//Set vocabulary parameter
	int offset = 0;
	std::string behaviorStr = generateXMLRPCArray(vocabVector);
	XmlRpc::XmlRpcValue val(behaviorStr, &offset);
	nh.setParam(VOCABULARY_KEY, val);

	//Reconfigure the nao_speech node
	ros::ServiceClient client = nh.serviceClient<std_srvs::Empty>("nao_speech/reconfigure");
	std_srvs::Empty emptySrv;

	client.call(emptySrv);

	std::cout << "Speech recognition has been successfully configured.\n";
}

std::string generateXMLRPCArray(std::vector<std::string> vector)
{
	std::ostringstream ss;

	ss << "<value><array><data>";

	for (int i=0;i<vector.size();i++){
		std::string& current = vector[i];

		ss << "<value><string>" << current << "</string></value>";
	}

	ss << "</value></array></data>";

	return ss.str();
}

void checkTfTransforms(){
	//Maybe do some kind of prompt here???

	//Create subscriber to the tf topic
	ros::NodeHandle handle;
	ros::Subscriber subscriber = handle.subscribe("/tf", 1, tfCallback);

	ros::Rate loopRate(60);

	ROS_INFO("Waiting for /tf topic publisher.");
	while (subscriber.getNumPublishers() == 0){
		if (!handle.ok()){
			ROS_INFO("Closing down.");

			exit(EXIT_FAILURE);
		}

		loopRate.sleep();
	}
	ROS_INFO("/tf topic publisher active.");

	ROS_INFO("Waiting for a head tf to be published.");
	while (!tfReady){
		ros::spinOnce();

		if (!handle.ok()){
			ROS_INFO("Closing down.");

			exit(EXIT_FAILURE);
		}

		loopRate.sleep();
	}

	ROS_INFO("Initialising done.");
}

void tfCallback(const tf::tfMessage msg)
{
	//Allow for application to continue when message containing tf for any head is recieved.
	geometry_msgs::TransformStamped transform = msg.transforms[0];
	if (transform.child_frame_id.find(HEAD_TF) != std::string::npos){
		ROS_INFO("/tf is now publishing, ready to go.");

		tfReady = true;
	}
}

bool checkRunningNodes(std::vector<std::string>& node_names)
{
	bool openniTrackerRunning = false;

	for (int i=0;i<node_names.size();i++){
		std::string& str = node_names[i];

		if (str == OPENNI_TRACKER){
			openniTrackerRunning = true;
		}
	}

	if (!openniTrackerRunning){
		std::cout << "OpenNI tracker node isn't running, attempting to start." << std::endl;

		std::system("rosrun openni_tracker openni_tracker &");

		std::cout << "Launched.";
	}else{
		return true;
	}

	return false;
}

std::map<std::string, std::vector<Phrase> > getPhraseMap(Json::Value& phraseRoot)
{
	std::map<std::string, std::vector<Phrase> > phraseMap;

	Json::Value::Members mems = phraseRoot.getMemberNames();
	Json::Value nullValue(Json::nullValue);

	for (int i=0;i<mems.size();i++){
		std::string key = mems[i];

		Json::Value val = phraseRoot.get(key, nullValue);
		//Sanity check, this should always be true
		if (val.type() != nullValue.type()){
			std::vector<Phrase> phraseVector;

			Json::Value phraseValues = val.get(PHRASE_KEY, nullValue);
			if (phraseValues.type() != nullValue.type()){
				//Load behaviors vector, if it is present in the file
				std::vector<std::string> behaviorVector;

				Json::Value behaviorValues = val.get(BEHAVIOR_KEY, nullValue);
				if (behaviorValues.type() != Json::Value::null.type()){
					Json::Value::ArrayIndex behaviorNum = behaviorValues.size();

						for (int j=0;j<behaviorNum;j++){
							Json::Value currentBehavior = behaviorValues.get(j, nullValue);

							if (currentBehavior.type() != Json::Value::null.type()){
								behaviorVector.push_back(currentBehavior.asString());
							}
						}
				}

				//Load all the possible phrases for the given phrase
				Json::Value::ArrayIndex size = val.size();

				for (int i=0;i<size;i++){
					Json::Value phraseValue = phraseValues.get(i, nullValue);

					//Again, sanity check, this should always be true
					if (phraseValue.type() != nullValue.type()){
						Phrase phrase(phraseValue.asString(), behaviorVector);

						phraseVector.push_back(phrase);
					}
				}
			}

			phraseMap.insert(std::pair<std::string, std::vector<Phrase> >(key, phraseVector));
		}
	}

	return phraseMap;
}

std::vector<Behavior> getBehaviorList(Json::Value& behaviorRoot)
{
	std::vector<Behavior> behaviorList;

	Json::Value::ArrayIndex size = behaviorRoot.size();
	for (int i=0;i<size;i++){
		Json::Value behavior = behaviorRoot.get(i, Json::Value::null);

		//Found behavior
		if (behavior.type() != Json::Value::null.type()){
			std::string name = "";
			std::string actual = "";
			int classification = 0;

			Json::Value behaviorActual = behavior.get(BEHAVIOR_ACTUAL_KEY, Json::Value::null);
			if (behaviorActual.type() != Json::Value::null.type()){
				actual = behaviorActual.asString();
			}

			Json::Value behaviorClassification = behavior.get(BEHAVIOR_CLASSIFICATION_KEY, Json::Value::null);
			if (behaviorClassification.type() != Json::Value::null.type()){
				classification = behaviorClassification.asInt();
			}

			Json::Value behaviorNames = behavior.get(BEHAVIOR_NAME_KEY, Json::Value::null);
			if (behaviorNames.type() != Json::Value::null.type()){
				//Now to iterate through array
				Json::Value::ArrayIndex innerSize = behaviorNames.size();

				for (int j=0;j<innerSize;j++){
					Json::Value val = behaviorNames.get(j, Json::Value::null);

					if (val.type() != Json::Value::null.type()){
						std::string name = val.asString();

						std::cout << "Loaded behavior (name: " << name << ", actual: " << actual
								<< ", classification: " << classification << ")\n";

						Behavior behaviorObj(name, actual, classification);
						behaviorList.push_back(behaviorObj);
					}
				}
			}
		}
	}

	return behaviorList;
}

bool loadPhraseMaps(Json::Value& doc, std::map<std::string, std::vector<Phrase> >& genericPhraseMap,
		std::map<std::string, std::vector<Phrase> >& guessGamePhraseMap,
		std::map<std::string, std::vector<Phrase> >& mimicGamePhraseMap)
{
	//Load phrase maps

	//Generate generic phrase map
	Json::Value genericPhrases = doc.get(PHRASE_KEY, Json::Value::null);
	if (genericPhrases.type() != Json::Value::null.type()){
		genericPhraseMap = getPhraseMap(genericPhrases);
	}else{
		ROS_ERROR("Unable to find generic phrases, perhaps the json data file is invalid, run the gen_json node to generate a new json file.");

		return false;
	}

	//Generate guess game phrase map
	Json::Value guessGameVal = doc.get(GUESS_GAME_KEY, Json::Value::null);
	if (guessGameVal.type() != Json::Value::null.type()){
		Json::Value guessGamePhrases = guessGameVal.get(PHRASE_KEY, Json::Value::null);

		if (guessGamePhrases != Json::Value::null.type()){
			guessGamePhraseMap = getPhraseMap(guessGamePhrases);
		}else{
			ROS_ERROR("Unable to find phrases for the guessing game, perhaps the json data file is invalid, run the gen_json node to generate a new json file.");

			return false;
		}
	}else{
		ROS_ERROR("Unable to find the guessing game data, perhaps the json data file is invalid, run the gen_json node to generate a new json file.");

		return false;
	}

	//Generate mimic game phrase map
	Json::Value mimicGameVal = doc.get(MIMIC_GAME_KEY, Json::Value::null);
	if (mimicGameVal.type() != Json::Value::null.type()){
		Json::Value mimicGamePhrases = mimicGameVal.get(PHRASE_KEY, Json::Value::null);

		if (mimicGamePhrases != Json::Value::null.type()){
			mimicGamePhraseMap = getPhraseMap(mimicGamePhrases);
		}else{
			ROS_ERROR("Unable to find phrases for the mimic game, perhaps the json data file is invalid, run the gen_json node to generate a new json file.");

			return false;
		}
	}else{
		ROS_ERROR("Unable to find the mimic game data, perhaps the json data file is invalid, run the gen_json node to generate a new json file.");

		return false;
	}

	return true;
}
