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

#define VOCABULARY_KEY "nao_speech/vocabulary"

#define NODE_NAME "emotion_game"
#define OPENNI_TRACKER "/openni_tracker"

#define HEAD_TF "head_"

bool tfReady;

void startSpeechRecognition();

bool checkRunningNodes(std::vector<std::string>&);
void checkTfTransforms();
void tfCallback(const tf::tfMessage);

std::map<std::string, std::list<Phrase> > getPhraseMap(Json::Value& phraseRoot);

int main(int argc, char** argv)
{
	if (argc != 2){
		std::cout << "Invalid arguments. The arguments should be as follows:\n"
				<< "\t<file>\n"
				<< "Where <file> is the location of the JSON data file.\n";

		return -1;
	}

	ros::init(argc, argv, NODE_NAME);

	std::vector<std::string> node_names;
	ros::master::getNodes(node_names);

	//Construct settings object
	GameSettings settings;

	settings.setWait(2);
	settings.setTimeout(5);

	//Read in json file
	char* fileName = argv[1];

	/*
	 *  std::string contents;
    in.seekg(0, std::ios::end);
    contents.resize(in.tellg());
    in.seekg(0, std::ios::beg);
    in.read(&contents[0], contents.size());
    in.close();
    return(contents);
	 */

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

		std::map<std::string, std::list<Phrase> > genericPhraseMap;
		std::map<std::string, std::list<Phrase> > guessGamePhraseMap;
		std::map<std::string, std::list<Phrase> > mimicGamePhraseMap;

		//Generate generic phrase map
		Json::Value genericPhrases = doc.get(PHRASE_KEY, nullValue);
		if (genericPhrases.type() != nullValue.type()){
			genericPhraseMap = getPhraseMap(genericPhrases);
		}

		//Generate guess game phrase map
		Json::Value guessGameVal = doc.get(GUESS_GAME_KEY, nullValue);
		if (guessGameVal.type() != nullValue.type()){
			Json::Value guessGamePhrases = guessGameVal.get(PHRASE_KEY, nullValue);

			if (guessGamePhrases != nullValue.type()){
				guessGamePhraseMap = getPhraseMap(guessGamePhrases);
			}else{
				ROS_ERROR("Unable to find phrases for the guessing game, perhaps the json data file is invalid, run the gen_json node to generate a new json file.");

				return 1;
			}
		}else{
			ROS_ERROR("Unable to find the guessing game data, perhaps the json data file is invalid, run the gen_json node to generate a new json file.");

			return -1;
		}

		//Generate mimic game phrase map
		Json::Value mimicGameVal = doc.get(MIMIC_GAME_KEY, nullValue);
		if (mimicGameVal.type() != nullValue.type()){
			Json::Value mimicGamePhrases = mimicGameVal.get(PHRASE_KEY, nullValue);

			if (mimicGamePhrases != nullValue.type()){
				mimicGamePhraseMap = getPhraseMap(mimicGamePhrases);
			}else{
				ROS_ERROR("Unable to find phrases for the mimic game, perhaps the json data file is invalid, run the gen_json node to generate a new json file.");

				return 1;
			}
		}else{
			ROS_ERROR("Unable to find the mimic game data, perhaps the json data file is invalid, run the gen_json node to generate a new json file.");

			return -1;
		}

		//Create settings and instances of games
		GameSettings guessGameSettings;
		guessGameSettings.setPhraseMap(guessGamePhraseMap);

		GameSettings mimicGameSettings;
		mimicGameSettings.setPhraseMap(mimicGamePhraseMap);

		Game* guessGame = new GuessGame(guessGameSettings);
		Game* mimicGame = new MimicGame(mimicGameSettings);

		//Set current game and start it.
		Game* currentGame = guessGame;
		currentGame->startGame();

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
		}
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

void startSpeechRecognition()
{
	ros::NodeHandle nh;

	while (!nh.ok());

	std::cout << "Sending parameters for speech recognition." << std::endl;

	//Set vocabulary parameter
	int offset = 0;
	XmlRpc::XmlRpcValue val("<value><array><data><value><string>hello</string></value><value><string>bye</string></value></data></array></value>", &offset);
	nh.setParam(VOCABULARY_KEY, val);

	std::cout << "XML: " << (val.getType() == XmlRpc::XmlRpcValue::TypeInvalid) << std::endl;
	std::cout << val.toXml() << std::endl;

	ros::ServiceClient client = nh.serviceClient<std_srvs::Empty>("nao_speech/reconfigure");
	std_srvs::Empty emptySrv;

	if (client.exists())
		std::cout << "Valid service." << "\n";

	client.call(emptySrv);

	client = nh.serviceClient<std_srvs::Empty>("nao_speech/start_recognition");

	if (client.exists())
		std::cout << "Valid service." << "\n";

	client.call(emptySrv);

	std::cout << "Speech recognition started." << std::endl;
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

std::map<std::string, std::list<Phrase> > getPhraseMap(Json::Value& phraseRoot)
{
	std::map<std::string, std::list<Phrase> > phraseMap;

	Json::Value::Members mems = phraseRoot.getMemberNames();
	Json::Value nullValue(Json::nullValue);

	for (int i=0;i<mems.size();i++){
		std::string key = mems[i];

		Json::Value val = phraseRoot.get(key, nullValue);
		//Sanity check, this should always be true
		if (val.type() != nullValue.type()){
			std::list<Phrase> phraseList;

			Json::Value::ArrayIndex size = val.size();

			for (int i=0;i<size;i++){
				Json::Value phraseValue = val.get(i, nullValue);

				//Again, sanity check, this should always be true
				if (phraseValue.type() != nullValue.type()){
					Phrase phrase(phraseValue.asString());

					phraseList.push_back(phrase);
				}
			}

			phraseMap.insert(std::pair<std::string, std::list<Phrase> >(key, phraseList));
		}
	}

	return phraseMap;
}
