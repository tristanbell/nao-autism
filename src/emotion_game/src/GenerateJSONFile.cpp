/*
 * This ROS node is used to generate the required JSON files.
 *
 * The node must be ran as follows:
 * 		rosrun emotion_game gen_json <file>
 * Where <file> is the path to the file that the json data will be
 * wrote to.
 */

#include <json/json.h>

#include <ros/ros.h>

#include <string>
#include <iostream>
#include <fstream>

#define BASE_SETTINGS_NAME "Base settings"

#define SPEECH_WAIT_SETTING_NAME "Speech wait"
#define DEFAULT_WAIT 3

#define TIMEOUT_SETTING "Timeout"
#define DEFAULT_TIMEOUT 5

#define BEHAVIOR_LIST_NAME "Available behaviors"
#define REWARD_BEHAVIOR_LIST "Available reward behaviors"

#define BEHAVIOR_NAME_KEY "Behavior name"
#define BEHAVIOR_ACTUAL_KEY "Actual name"

#define HAPPY_BEHAVIOR_NAME "happy_1"
#define HAPPY_BEHAVIOR_ACTUAL "happy"

#define SCARED_BEHAVIOR_NAME "scared_1"
#define SCARED_BEHAVIOR_ACTUAL "scared"

#define GUESS_GAME_SETTINGS_NAME "Guess game settings"

#define MIMIC_GAME_SETTINGS_NAME "Mimic game settings"

Json::Value generateBehaviorList();

int main(int argc, char** argv)
{
	ros::init(argc, argv, "json_gen");

	if (argc == 2){
		std::cout << "Generating JSON data." << std::endl;

		Json::Value doc(Json::objectValue);

		//Generate the base settings
		Json::Value& baseSettings = doc[BASE_SETTINGS_NAME];
		Json::Value& waitSetting = baseSettings[SPEECH_WAIT_SETTING_NAME];
		waitSetting = DEFAULT_WAIT;

		Json::Value& timeoutSetting = baseSettings[TIMEOUT_SETTING];
		timeoutSetting = DEFAULT_TIMEOUT;

		//Generate guess game settings
		Json::Value& guessGameSettings = doc[GUESS_GAME_SETTINGS_NAME];

		//Generate mimic game settings
		Json::Value& mimicGameSettings = doc[MIMIC_GAME_SETTINGS_NAME];

		//Generate behavior list
		Json::Value& behaviorList = doc[BEHAVIOR_LIST_NAME];
		behaviorList = generateBehaviorList();

		Json::StyledWriter writer;
		std::string result = writer.write(doc);

		std::cout << "Generation finished." << std::endl;

		//Write json data to file
		std::fstream fs;
		fs.open(argv[1], std::fstream::out);

		std::cout << "Writing json data to " << argv[1] << std::endl;

		//Write the json data and flush to ensure complete write
		fs << result;
		fs.flush();

		std::cout << "Successfully wrote generated json data.";

		//Close stream
		fs.close();
	}else{
		std::cout << "Invalid number of arguments, the following format must be used:" << std::endl;
		std::cout << "\trosrun emotion_game gen_json <file>" << std::endl;
		std::cout << "Where <file> is the file name to write the json data to." << std::endl;
	}

	ros::shutdown();

	return 0;
}

/**
 * Generates a Json value object containing all the required behavior information.
 *
 * The list is a Json array with each entry being a Json object containing the following key-value pairs.
 * 		Behavior name
 * 		Actual behavior name
 * 		Classification (if the value is -1, then there is no classification)
 *
 * Note: The classification is hard-coded at this stage as there is no way to make it completely dynamic.
 * Although this approach means that it is possible to include extra behaviors in the game without much work.
 */
Json::Value generateBehaviorList()
{
	Json::Value list(Json::arrayValue);

	Json::Value happyBehavior(Json::objectValue);

	Json::Value& happyName = happyBehavior[BEHAVIOR_NAME_KEY];
	happyName = HAPPY_BEHAVIOR_NAME;

	Json::Value& happyActual = happyBehavior[BEHAVIOR_ACTUAL_KEY];
	happyActual = HAPPY_BEHAVIOR_ACTUAL;

	Json::Value scaredBehavior(Json::objectValue);

	Json::Value& scaredName = scaredBehavior[BEHAVIOR_NAME_KEY];
	scaredName = SCARED_BEHAVIOR_NAME;

	Json::Value& scaredActual = scaredBehavior[BEHAVIOR_ACTUAL_KEY];
	scaredActual = SCARED_BEHAVIOR_ACTUAL;

	list.append(happyBehavior);
	list.append(scaredBehavior);

	return list;
}


