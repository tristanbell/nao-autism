/*
 * This ROS node is used to generate the required JSON files.
 */

#include <json/json.h>

#include <ros/ros.h>

#include <string>
#include <iostream>

#define BASE_SETTINGS_NAME "Base settings"

#define SPEECH_WAIT_SETTING_NAME "Speech wait"
#define DEFAULT_WAIT 3

#define TIMEOUT_SETTING "Timeout"
#define DEFAULT_TIMEOUT 5

#define BEHAVIOR_LIST_NAME "Available behaviors"

#define BEHAVIOR_ACTUAL_KEY "Actual name"

#define HAPPY_BEHAVIOR_NAME "happy_1"
#define HAPPY_BEHAVIOR_ACTUAL "happy"

#define GUESS_GAME_SETTINGS_NAME "Guess game settings"

#define MIMIC_GAME_SETTINGS_NAME "Mimic game settings"

Json::Value generateBehaviorList();

int main(int argc, char** argv)
{
	ros::init(argc, argv, "json_gen");

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

	Json::Value& behaviorList = guessGameSettings[BEHAVIOR_LIST_NAME];

	behaviorList = generateBehaviorList();

	//Generate mimic game settings
	Json::Value& mimicGameSettings = doc[MIMIC_GAME_SETTINGS_NAME];

	Json::StyledWriter writer;
	std::string result = writer.write(doc);

	std::cout << result << std::endl;

	return 0;
}

Json::Value generateBehaviorList()
{
	Json::Value list(Json::objectValue);

	Json::Value& happyBehavior = list[HAPPY_BEHAVIOR_NAME];
	Json::Value& happyActual = happyBehavior[BEHAVIOR_ACTUAL_KEY];
	happyActual = HAPPY_BEHAVIOR_ACTUAL;

	return list;
}


