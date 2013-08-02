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

#include <Keys.h>

#define DEFAULT_WAIT 3
#define DEFAULT_TIMEOUT 15
#define DEFAULT_PROMPTS 2

#define HAPPY_BEHAVIOR_NAME "happy_1"
#define HAPPY_BEHAVIOR_ACTUAL "happy"
#define HAPPY_BEHAVIOR_CLASSIFICATION 1

#define SCARED_BEHAVIOR_NAME "scared_1"
#define SCARED_BEHAVIOR_ACTUAL "scared"
#define SCARED_BEHAVIOR_CLASSIFICATION 2

#define SAD_BEHAVIOR_NAME "sad_1"
#define SAD_BEHAVIOR_ACTUAL "sad"
#define SAD_BEHAVIOR_CLASSIFICATION 3

#define ANGRY_BEHAVIOR_NAME "angry_1"
#define ANGRY_BEHAVIOR_ACTUAL "angry"
#define ANGRY_BEHAVIOR_CLASSIFICATION 4

#define REWARD_1_BEHAVIOR_NAME "reward_1"
#define REWARD_2_BEHAVIOR_NAME "reward_2"
#define REWARD_3_BEHAVIOR_NAME "reward_3"

#define KINECT_PROMPT_PHRASE "Copy the robot"
#define POSITIVE_PHRASE "Well done"
#define INTRODUCTION_PHRASE "Lets play"

#define GUESS_INTRODUCTION_PHRASE "Guess the emotion"
#define GUESS_INSTRUCTION_PHRASE "The robot will do an emotion and you will have to guess what it is"
#define GUESS_NEXT_PHRASE "Next emotion"
#define GUESS_QUESTION_1 "Was the robot % or %?"
#define GUESS_CORRECT_ANSWER_1 "Well done, you guessed the robot was %"
#define GUESS_PROMPT_ANSWER_1 "Try again"
#define GUESS_INCORRECT_ANSWER_1 "Lets try another emotion"
#define GUESS_FINISH_PHRASE "Guess the emotion is finished"

#define MIMIC_INTRODUCTION_PHRASE "Copy the robot"
#define MIMIC_INSTRUCTION_PHRASE "The robot will do an emotion and you will have to copy the emotion when the robot asks you to"
#define MIMIC_EMOTION_PHRASE "The robot is %"
#define MIMIC_PROMPT_FOLLOW_PHRASE "Do the same"
#define MIMIC_CORRECT_PHRASE "Well done"
#define MIMIC_PROMPT_PHRASE "Try again"
#define MIMIC_INCORRECT_PHRASE "Better luck next time"
#define MIMIC_FINISH_PHRASE "Copy the robot is finished."

Json::Value generateBehaviorList();
Json::Value generateRewardBehaviorsList();

void generateBehavior(Json::Value&, const std::string, const std::string, int);

Json::Value generateGenericPhrases();
Json::Value generateGuessGamePhrases();
Json::Value generateMimicGamePhrases();

int main(int argc, char** argv)
{
	ros::init(argc, argv, "json_gen");

	if (argc == 2){
		std::cout << "Generating JSON data." << std::endl;

		Json::Value doc(Json::objectValue);

		//Generate the base settings
		Json::Value& baseSettings = doc[BASE_SETTINGS_KEY];
		Json::Value& waitSetting = baseSettings[SPEECH_WAIT_SETTING_KEY];
		waitSetting = DEFAULT_WAIT;

		Json::Value& timeoutSetting = baseSettings[TIMEOUT_SETTING_KEY];
		timeoutSetting = DEFAULT_TIMEOUT;

		Json::Value& maxTimesPrompted = baseSettings[MAX_PROMPT_KEY];
		maxTimesPrompted = DEFAULT_PROMPTS;

		//Generate generic phrases
		Json::Value& genericPhrases = doc[PHRASE_KEY];
		genericPhrases = generateGenericPhrases();

		//Generate guess game settings
		Json::Value& guessGameSettings = doc[GUESS_GAME_KEY];

		//Generate phrases for guess game
		Json::Value& guessGamePhrases = guessGameSettings[PHRASE_KEY];
		guessGamePhrases = generateGuessGamePhrases();

		//Generate mimic game settings
		Json::Value& mimicGameSettings = doc[MIMIC_GAME_KEY];

		//Generate phrase for mimic game
		Json::Value& mimicGamePhrases = mimicGameSettings[PHRASE_KEY];
		mimicGamePhrases = generateMimicGamePhrases();

		//Generate behavior list
		Json::Value& behaviorList = doc[BEHAVIOR_LIST_KEY];
		behaviorList = generateBehaviorList();

		//Generate reward behavior list
		Json::Value& rewardBehaviorList = doc[REWARD_BEHAVIOR_LIST_KEY];
		rewardBehaviorList = generateRewardBehaviorsList();

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
	generateBehavior(happyBehavior, HAPPY_BEHAVIOR_NAME, HAPPY_BEHAVIOR_ACTUAL, HAPPY_BEHAVIOR_CLASSIFICATION);

	Json::Value scaredBehavior(Json::objectValue);
	generateBehavior(scaredBehavior, SCARED_BEHAVIOR_NAME, SCARED_BEHAVIOR_ACTUAL, SCARED_BEHAVIOR_CLASSIFICATION);

	Json::Value sadBehavior(Json::objectValue);
	generateBehavior(sadBehavior, SAD_BEHAVIOR_NAME, SAD_BEHAVIOR_ACTUAL, SAD_BEHAVIOR_CLASSIFICATION);

	Json::Value angryBehavior(Json::objectValue);
	generateBehavior(angryBehavior, ANGRY_BEHAVIOR_NAME, ANGRY_BEHAVIOR_ACTUAL, ANGRY_BEHAVIOR_CLASSIFICATION);

	list.append(happyBehavior);
	list.append(scaredBehavior);
	list.append(sadBehavior);
	list.append(angryBehavior);

	return list;
}

void generateBehavior(Json::Value& root, const std::string name, const std::string actual, int classification)
{
	Json::Value& nameVal = root[BEHAVIOR_NAME_KEY];
	nameVal.append(name);

	Json::Value& actualVal = root[BEHAVIOR_ACTUAL_KEY];
	actualVal = actual;

	Json::Value& classificationVal = root[BEHAVIOR_CLASSIFICATION_KEY];
	classificationVal = classification;
}

Json::Value generateRewardBehaviorsList()
{
	Json::Value list(Json::arrayValue);

	Json::Value reward_1(Json::objectValue);

	Json::Value& reward_1_name = reward_1[BEHAVIOR_NAME_KEY];
	reward_1_name.append(REWARD_1_BEHAVIOR_NAME);
	reward_1_name.append(REWARD_2_BEHAVIOR_NAME);
	reward_1_name.append(REWARD_3_BEHAVIOR_NAME);

	list.append(reward_1);

	return list;
}

Json::Value generateGenericPhrases()
{
	Json::Value val(Json::objectValue);

	Json::Value& promptPhrase = val[KINECT_PROMPT_KEY];
	promptPhrase.append(KINECT_PROMPT_PHRASE);

	Json::Value& positivePhrase = val[POSITIVE_KEY];
	positivePhrase.append(POSITIVE_PHRASE);

	Json::Value& introductionPhrase = val[INTRODUCTION_KEY];
	introductionPhrase.append(INTRODUCTION_PHRASE);

	return val;
}

Json::Value generateGuessGamePhrases()
{
	Json::Value val(Json::objectValue);

	Json::Value& startPhrase = val[START_KEY];
	startPhrase.append(GUESS_INTRODUCTION_PHRASE);

	Json::Value& instructionPhrase = val[INSTRUCTION_KEY];
	instructionPhrase.append(GUESS_INSTRUCTION_PHRASE);

	Json::Value& nextEmotionPhrase = val[GUESS_NEXT_KEY];
	nextEmotionPhrase.append(GUESS_NEXT_PHRASE);

	Json::Value& questionPhrases = val[QUESTION_KEY];
	questionPhrases.append(GUESS_QUESTION_1);

	Json::Value& correctPhrases = val[CORRECT_ANSWER_KEY];
	correctPhrases.append(GUESS_CORRECT_ANSWER_1);

	Json::Value& promptPhrases = val[PROMPT_KEY];
	promptPhrases.append(GUESS_PROMPT_ANSWER_1);

	Json::Value& incorrectPhrases = val[INCORRECT_ANSWER_KEY];
	incorrectPhrases.append(GUESS_INCORRECT_ANSWER_1);

	Json::Value& finishPhrase = val[FINISH_KEY];
	finishPhrase.append(GUESS_FINISH_PHRASE);

	return val;
}

Json::Value generateMimicGamePhrases()
{
	Json::Value val(Json::objectValue);

	Json::Value& startPhrase = val[START_KEY];
	startPhrase.append(MIMIC_INTRODUCTION_PHRASE);

	Json::Value& instructionPhrase = val[INSTRUCTION_KEY];
	instructionPhrase.append(INSTRUCTION_KEY);

	Json::Value& emotionPhrase = val[MIMIC_EMOTION_KEY];
	emotionPhrase.append(MIMIC_EMOTION_PHRASE);

	Json::Value& promptFollowPhrase = val[MIMIC_PROMPT_FOLLOW_KEY];
	promptFollowPhrase.append(MIMIC_PROMPT_FOLLOW_PHRASE);

	Json::Value& correctPhrase = val[CORRECT_ANSWER_KEY];
	correctPhrase.append(MIMIC_CORRECT_PHRASE);

	Json::Value& incorrectPhrase = val[INCORRECT_ANSWER_KEY];
	incorrectPhrase.append(MIMIC_INCORRECT_PHRASE);

	Json::Value& finishPhrase = val[FINISH_KEY];
	finishPhrase.append(MIMIC_FINISH_PHRASE);

	return val;
}
