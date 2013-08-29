/*
 * GenerateJSONFile.h
 *
 *  Created on: 23 Aug 2013
 *      Author: alex
 */

#ifndef GENERATEJSONFILE_H_
#define GENERATEJSONFILE_H_

#include <json/json.h>

#include <string>
#include <iostream>
#include <fstream>

#include <Keys.h>

#define DEFAULT_WAIT 3
#define DEFAULT_TIMEOUT 15
#define DEFAULT_PROMPTS 2
#define DEFAULT_EMOTIONS_QUESTION 3
#define DEFAULT_CONFIDENCE 0.3f

#define HAPPY_BEHAVIOR_NAME "happy_1"
#define HAPPY_BEHAVIOR_ACTUAL "happy"
#define HAPPY_BEHAVIOR_CLASSIFICATION 1

#define SAD_BEHAVIOR_NAME "sad_1"
#define SAD_BEHAVIOR_ACTUAL "sad"
#define SAD_BEHAVIOR_CLASSIFICATION 2

#define SCARED_BEHAVIOR_NAME "scared_1"
#define SCARED_BEHAVIOR_ACTUAL "scared"
#define SCARED_BEHAVIOR_CLASSIFICATION 3

#define ANGRY_BEHAVIOR_NAME "angry_1"
#define ANGRY_BEHAVIOR_ACTUAL "angry"
#define ANGRY_BEHAVIOR_CLASSIFICATION 4

#define REWARD_1_BEHAVIOR_NAME "reward_1"
#define REWARD_2_BEHAVIOR_NAME "reward_2"
#define REWARD_3_BEHAVIOR_NAME "reward_3"

#define MIMIC_PROMPT_COPY_BEHAVIOR "arms_up"

#define KINECT_PROMPT_PHRASE "Copy the robot"
#define POSITIVE_PHRASE "Well done"
#define INTRODUCTION_PHRASE "Lets play"
#define REWARD_PROMPT_PHRASE "Lets dance"

#define GUESS_INTRODUCTION_PHRASE "Guess the emotion"
#define GUESS_INSTRUCTION_PHRASE "The robot will do an emotion and you will have to guess what it is"
#define GUESS_NEXT_PHRASE "Next emotion"
#define GUESS_QUESTION_1 "Was the robot % or %?"
#define GUESS_CORRECT_ANSWER_1 "Well done, you guessed the robot was %"
#define GUESS_PROMPT_ANSWER_1 "Try again"
#define GUESS_INCORRECT_ANSWER_1 "Lets try another emotion"
#define GUESS_CONTINUE_QUESTION_1 "Shall we play the guessing game again?"
#define GUESS_FINISH_PHRASE "Guess the emotion is finished"

#define MIMIC_INTRODUCTION_PHRASE "Copy the robot"
#define MIMIC_INSTRUCTION_PHRASE "The robot will do an emotion and you will have to copy the emotion when the robot asks you to"
#define MIMIC_EMOTION_PHRASE "The robot is %"
#define MIMIC_PROMPT_FOLLOW_PHRASE "Do the same"
#define MIMIC_PROMPT_COPY_PHRASE "Do what the robot is doing"
#define MIMIC_CORRECT_PHRASE "Well done"
#define MIMIC_PROMPT_PHRASE "Try again"
#define MIMIC_INCORRECT_PHRASE "Better luck next time"
#define MIMIC_CONTINUE_QUESTION_1 "Shall we play the mimic game again?"
#define MIMIC_FINISH_PHRASE "Copy the robot is finished."

#define QUESTION_PHRASE_BEHAVIOR_1 "prompt_1"
#define INFORM_PHRASE_BEHAVIOR_1 "prompt_2"
#define CORRECT_ANSWER_BEHAVIOR_1 "right_1"
#define INCORRECT_ANSWER_BEHAVIOR_1 "incorrect_1"

Json::Value generateAllData();

Json::Value generateBehaviorList();
Json::Value generateRewardBehaviorsList();

void generateBehavior(Json::Value&, const std::string, const std::string, int);

Json::Value generateGenericPhrases();
Json::Value generateGuessGamePhrases();
Json::Value generateMimicGamePhrases();

#endif /* GENERATEJSONFILE_H_ */
