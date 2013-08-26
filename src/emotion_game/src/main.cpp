/*
 * main.cpp
 *
 *  Created on: 29 Jul 2013
 *      Author: tristan
 */

#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Empty.h>

#include <nao_autism_messages/ExecutionStatus.h>
#include <nao_autism_messages/SetExecutionStatus.h>

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

#define SAFE_POS_BEHAVIOR "sit_down"
#define INIT_BEHAVIOR "init"

#define VOCABULARY_KEY "nao_speech/vocabulary"

#define NODE_NAME "emotion_game"
#define OPENNI_TRACKER "/openni_tracker"

#define HEAD_TF "head_"

bool tfReady;

int speechWait;

std::map<std::string, std::vector<Phrase> > genericPhraseMap;
std::vector<Behavior> rewardBehaviorList;

Game* guessGame;
Game* mimicGame;

//Variables required to handle the overriding of the game loop if required
enum ExecutionStatus{
	GAME_RUNNING,
	GAME_PAUSED,
	GAME_STOPPED
};

ExecutionStatus executionStatus;

ros::Subscriber naoStopSubscriber;

ros::Subscriber setExecutionStatusSubscriber;
ros::Publisher executionStatusPublisher;

//Methods required for running the game
void runGameLoop();
void rewardChild(nao_control::NaoControl& cntrl);

//Methods required for initialising speech recognition
void initSpeechRecognition(std::vector<std::string>& vocabVector);
std::string generateXMLRPCArray(std::vector<std::string> vector);

//Methods required for checking if everything is running
bool checkRunningNodes(std::vector<std::string>&);
void checkTfTransforms();
void tfCallback(const tf::tfMessage);

/**
 * This method will override control of the game loop and stop the Nao safely, this
 * is used to prevent damage to the Nao because of overheating joints/low battery.
 */
void stopNaoCallback(const std_msgs::Empty&);

void setExecutionStatusCallback(const nao_autism_messages::SetExecutionStatus&);

//Methods required for loading settings
std::vector<Behavior> getBehaviorList(Json::Value& behaviorRoot);

std::map<std::string, std::vector<Phrase> > getPhraseMap(Json::Value& phraseRoot);
bool loadPhraseMaps(Json::Value& doc, std::map<std::string, std::vector<Phrase> >& genericPhraseMap,
		std::map<std::string, std::vector<Phrase> >& guessGamePhraseMap,
		std::map<std::string, std::vector<Phrase> >& mimicGamePhraseMap);

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
		speechWait = 3;
		int timeout = 5;
		int maxPrompts = 2;
		float confidenceValue = 0.3;

		std::vector<Behavior> allBehaviorList;

		std::map<std::string, std::vector<Phrase> > guessGamePhraseMap, mimicGamePhraseMap;
		if (!loadPhraseMaps(doc, genericPhraseMap, guessGamePhraseMap, mimicGamePhraseMap))
			return 1;

		//Load speech/timeout wait variables
		Json::Value baseSettings = doc.get(BASE_SETTINGS_KEY, nullValue);
		if (baseSettings.type() != nullValue.type()){
			Json::Value waitVal = baseSettings.get(SPEECH_WAIT_SETTING_KEY, nullValue);
			if (waitVal.type() != nullValue.type()){
				speechWait = waitVal.asInt();
			}

			Json::Value timeoutVal = baseSettings.get(TIMEOUT_SETTING_KEY, nullValue);
			if (timeoutVal.type() != nullValue.type()){
				timeout = timeoutVal.asInt();
			}

			Json::Value maxPromptVal = baseSettings.get(MAX_PROMPT_KEY, Json::Value::null);
			if (maxPromptVal.type() != Json::Value::null.type()){
				maxPrompts = maxPromptVal.asInt();
			}

			Json::Value confidenceVal = baseSettings.get(SPEECH_RECOGNITION_CONFIDENCE_KEY, Json::Value::null);
			if (confidenceVal.type() != Json::Value::null.type()){
				confidenceValue = confidenceVal.asFloat();
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

		//Create subscriber to allow for overriding game control and stop the nao
		ros::NodeHandle nh("~");
		naoStopSubscriber = nh.subscribe("stop", 100, stopNaoCallback); //This will be changed in the future

		setExecutionStatusSubscriber = nh.subscribe("set_execution_status", 1, setExecutionStatusCallback);
		executionStatusPublisher = nh.advertise<nao_autism_messages::ExecutionStatus>("execution_status", 1);

		//Create settings and instances of games
		GameSettings guessGameSettings;

		guessGameSettings.setWait(speechWait);
		guessGameSettings.setTimeout(timeout);
		guessGameSettings.setMaxPromptAmount(maxPrompts);
		guessGameSettings.setBehaviorVector(allBehaviorList);
		guessGameSettings.setPhraseMap(guessGamePhraseMap);
		guessGameSettings.setConfidenceThreshold(confidenceValue);

		GameSettings mimicGameSettings;

		mimicGameSettings.setWait(speechWait);
		mimicGameSettings.setTimeout(timeout);
		mimicGameSettings.setBehaviorVector(allBehaviorList);
		mimicGameSettings.setMaxPromptAmount(maxPrompts);
		mimicGameSettings.setPhraseMap(mimicGamePhraseMap);
		mimicGameSettings.setConfidenceThreshold(confidenceValue);

		guessGame = new GuessGame(guessGameSettings);
		mimicGame = new MimicGame(mimicGameSettings);

		ROS_INFO("Game initialisation done, starting.");

		runGameLoop();
	}else{
		std::cout << "Invalid json data file." << std::endl;

		return 1;
	}

	return 0;
}

void runGameLoop()
{
	nao_control::NaoControl rewardBehaviorControl;

	rewardBehaviorControl.perform("stand_up");
	rewardBehaviorControl.perform("init");

	//Set current game and start it.
	Game* currentGame = guessGame;
	currentGame->startGame();

	ros::Rate loopRate(40);
	//All checks are done, start game loop
	while (ros::ok()){
		if (executionStatus == GAME_STOPPED){
			//Send stopping message
			nao_autism_messages::ExecutionStatus msg;
			msg.status = nao_autism_messages::ExecutionStatus::STOPPING;
			executionStatusPublisher.publish(msg);

			std::cout << "Recieved stop message, stopping the nao and returning to safe position\n";

			rewardBehaviorControl.say("I don't feel too well, I am going to sit down.");

			rewardBehaviorControl.perform(INIT_BEHAVIOR);
			rewardBehaviorControl.perform(SAFE_POS_BEHAVIOR);

			break;
		}else if (executionStatus == GAME_PAUSED){
			//Send paused message
			nao_autism_messages::ExecutionStatus msg;
			msg.status = nao_autism_messages::ExecutionStatus::PAUSED;
			executionStatusPublisher.publish(msg);

			loopRate.sleep();
		}else{
			//Send running message
			nao_autism_messages::ExecutionStatus msg;
			msg.status = nao_autism_messages::ExecutionStatus::RUNNING;
			executionStatusPublisher.publish(msg);

			if (!currentGame->isDone){
				currentGame->perform();
			}else{
				//Reward child after game has finished
				rewardChild(rewardBehaviorControl);

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
		}

		//Spin once to enable call backs, etc.
		ros::spinOnce();
		loopRate.sleep();
	}

	//Force speech recognition to stop
	currentGame->stopSpeechRecognition();
}

void rewardChild(nao_control::NaoControl& cntrl)
{
	//Prompt child about reward
	try{
		//Get random phrase and say it
		std::vector<Phrase>& phraseVector = genericPhraseMap.at(REWARD_PROMPT_KEY);
		int rnd = rand() % phraseVector.size();

		Phrase& phrase = phraseVector[rnd];
		cntrl.say(phrase.getPhrase());
		sleep(speechWait);

		//Find random reward behavior and perform it
		rnd = rand() % rewardBehaviorList.size();
		Behavior& behavior = rewardBehaviorList[rnd];

		cntrl.perform(behavior.getName());

		//Say positive phrase to child
		phraseVector = genericPhraseMap.at(POSITIVE_KEY);
		rnd = rand() % phraseVector.size();

		phrase = phraseVector[rnd];
		cntrl.say(phrase.getPhrase());
		sleep(speechWait);
	}catch (std::out_of_range& oor){

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

void stopNaoCallback(const std_msgs::Empty& emptyMsg)
{
	executionStatus = GAME_STOPPED;
}

void setExecutionStatusCallback(const nao_autism_messages::SetExecutionStatus& msg)
{
	//Change execution status of the game based on what status has been sent in the message
	switch (msg.newStatus){

		case nao_autism_messages::SetExecutionStatus::ACTIVE:{
			executionStatus = GAME_RUNNING;

			break;
		}

		case nao_autism_messages::SetExecutionStatus::INACTIVE:{
			executionStatus = GAME_PAUSED;

			break;
		}

		case nao_autism_messages::SetExecutionStatus::STOP:{
			executionStatus = GAME_STOPPED;

			break;
		}

	}
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
