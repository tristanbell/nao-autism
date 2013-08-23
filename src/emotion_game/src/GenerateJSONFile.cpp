#include <GenerateJSONFile.h>

#include <json/json.h>

#include <ros/ros.h>

#include <string>
#include <iostream>
#include <fstream>

#include <Keys.h>

Json::Value generateAllData()
{
	Json::Value doc(Json::objectValue);

	//Generate the base settings
	Json::Value& baseSettings = doc[BASE_SETTINGS_KEY];
	Json::Value& waitSetting = baseSettings[SPEECH_WAIT_SETTING_KEY];
	waitSetting = DEFAULT_WAIT;

	Json::Value& timeoutSetting = baseSettings[TIMEOUT_SETTING_KEY];
	timeoutSetting = DEFAULT_TIMEOUT;

	Json::Value& maxTimesPrompted = baseSettings[MAX_PROMPT_KEY];
	maxTimesPrompted = DEFAULT_PROMPTS;

	Json::Value& confidence = baseSettings[SPEECH_RECOGNITION_CONFIDENCE_KEY];
	confidence = DEFAULT_CONFIDENCE;

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
	Json::Value& behaviorList = doc[BEHAVIOR_KEY];
	behaviorList = generateBehaviorList();

	//Generate reward behavior list
	Json::Value& rewardBehaviorList = doc[REWARD_BEHAVIOR_LIST_KEY];
	rewardBehaviorList = generateRewardBehaviorsList();

	return doc;
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
	Json::Value reward_1(Json::objectValue);

	Json::Value& reward_1_name = reward_1[BEHAVIOR_NAME_KEY];
	reward_1_name.append(REWARD_1_BEHAVIOR_NAME);
	reward_1_name.append(REWARD_2_BEHAVIOR_NAME);
	reward_1_name.append(REWARD_3_BEHAVIOR_NAME);

	return reward_1;
}

Json::Value generateGenericPhrases()
{
	Json::Value val(Json::objectValue);

	Json::Value& promptPhrase = val[KINECT_PROMPT_KEY];
	Json::Value& promptPhrases = promptPhrase[PHRASE_KEY];
	promptPhrases.append(KINECT_PROMPT_PHRASE);
	Json::Value& promptBehaviors = promptPhrase[BEHAVIOR_KEY];
	promptBehaviors.append(QUESTION_PHRASE_BEHAVIOR_1);

	Json::Value& positivePhrase = val[POSITIVE_KEY];
	Json::Value& positivePhrases = positivePhrase[PHRASE_KEY];
	positivePhrases.append(POSITIVE_PHRASE);
	Json::Value& positiveBehaviors = positivePhrase[BEHAVIOR_KEY];
	positiveBehaviors.append(CORRECT_ANSWER_BEHAVIOR_1);

	Json::Value& introductionPhrase = val[INTRODUCTION_KEY];
	Json::Value& introductionPhrases = introductionPhrase[PHRASE_KEY];
	introductionPhrases.append(INTRODUCTION_PHRASE);
	Json::Value& introductionBehaviors = introductionPhrase[BEHAVIOR_KEY];
	introductionBehaviors.append(INFORM_PHRASE_BEHAVIOR_1);

	Json::Value& rewardPromptPhrase = val[REWARD_PROMPT_KEY];
	Json::Value& rewardPromptPhrases = rewardPromptPhrase[PHRASE_KEY];
	rewardPromptPhrases.append(REWARD_PROMPT_PHRASE);
	Json::Value& rewardPromptBehaviors = rewardPromptPhrase[BEHAVIOR_KEY];

	return val;
}

Json::Value generateGuessGamePhrases()
{
	Json::Value val(Json::objectValue);

	Json::Value& startPhrase = val[START_KEY];
	Json::Value& startPhrases = startPhrase[PHRASE_KEY];
	startPhrases.append(GUESS_INTRODUCTION_PHRASE);
	Json::Value& startBehaviors = startPhrase[BEHAVIOR_KEY];
	startBehaviors.append(INFORM_PHRASE_BEHAVIOR_1);

	Json::Value& instructionPhrase = val[INSTRUCTION_KEY];
	Json::Value& instructionPhrases = instructionPhrase[PHRASE_KEY];
	instructionPhrases.append(GUESS_INSTRUCTION_PHRASE);
	Json::Value& instructionBehaviors =	instructionPhrase[BEHAVIOR_KEY];
	instructionBehaviors.append(INFORM_PHRASE_BEHAVIOR_1);

	Json::Value& nextEmotionPhrase = val[GUESS_NEXT_KEY];
	Json::Value& nextEmotionPhrases = nextEmotionPhrase[PHRASE_KEY];
	nextEmotionPhrases.append(GUESS_NEXT_PHRASE);
	Json::Value& nextEmotionBehaviors = nextEmotionPhrase[BEHAVIOR_KEY];
	nextEmotionBehaviors.append(INFORM_PHRASE_BEHAVIOR_1);

	Json::Value& questionPhrase = val[QUESTION_KEY];
	Json::Value& questionPhrases = questionPhrase[PHRASE_KEY];
	questionPhrases.append(GUESS_QUESTION_1);
	Json::Value& questionBehaviors = questionPhrase[BEHAVIOR_KEY];
	questionBehaviors.append(QUESTION_PHRASE_BEHAVIOR_1);

	Json::Value& correctPhrase = val[CORRECT_ANSWER_KEY];
	Json::Value& correctPhrases = correctPhrase[PHRASE_KEY];
	correctPhrases.append(GUESS_CORRECT_ANSWER_1);
	Json::Value& correctBehaviors = correctPhrase[BEHAVIOR_KEY];
	correctBehaviors.append(CORRECT_ANSWER_BEHAVIOR_1);

	Json::Value& promptPhrase = val[PROMPT_KEY];
	Json::Value& promptPhrases = promptPhrase[PHRASE_KEY];
	promptPhrases.append(GUESS_PROMPT_ANSWER_1);
	Json::Value& promptBehaviors = promptPhrase[BEHAVIOR_KEY];
	promptBehaviors.append(INFORM_PHRASE_BEHAVIOR_1);

	Json::Value& incorrectPhrase = val[INCORRECT_ANSWER_KEY];
	Json::Value& incorrectPhrases = incorrectPhrase[PHRASE_KEY];
	incorrectPhrases.append(GUESS_INCORRECT_ANSWER_1);
	Json::Value& incorrectBehaviors = incorrectPhrase[BEHAVIOR_KEY];
	incorrectBehaviors.append(INCORRECT_ANSWER_BEHAVIOR_1);

	Json::Value& continueGameQuestionPhrase = val[CONTINUE_GAME_QUESTION_KEY];
	Json::Value& continueGameQuestionPhrases = continueGameQuestionPhrase[PHRASE_KEY];
	continueGameQuestionPhrases.append(GUESS_CONTINUE_QUESTION_1);
	Json::Value& continueGameQuestionBehavior = continueGameQuestionPhrase[BEHAVIOR_KEY];
	continueGameQuestionBehavior.append(QUESTION_PHRASE_BEHAVIOR_1);

	Json::Value& finishPhrase = val[FINISH_KEY];
	Json::Value& finishPhrases = finishPhrase[PHRASE_KEY];
	finishPhrases.append(GUESS_FINISH_PHRASE);
	Json::Value& finishBehavior = finishPhrase[BEHAVIOR_KEY];
	finishBehavior.append(INFORM_PHRASE_BEHAVIOR_1);

	return val;
}

Json::Value generateMimicGamePhrases()
{
	Json::Value val(Json::objectValue);

	Json::Value& startPhrase = val[START_KEY];
	Json::Value& startPhrases = startPhrase[PHRASE_KEY];
	startPhrases.append(MIMIC_INTRODUCTION_PHRASE);
	Json::Value& startBehaviors = startPhrase[BEHAVIOR_KEY];
	startBehaviors.append(QUESTION_PHRASE_BEHAVIOR_1);

	Json::Value& instructionPhrase = val[INSTRUCTION_KEY];
	Json::Value& instructionPhrases = instructionPhrase[PHRASE_KEY];
	instructionPhrases.append(MIMIC_INSTRUCTION_PHRASE);
	Json::Value& instructionBehaviors = instructionPhrase[BEHAVIOR_KEY];
	instructionBehaviors.append(INFORM_PHRASE_BEHAVIOR_1);

	Json::Value& emotionPhrase = val[MIMIC_EMOTION_KEY];
	Json::Value& emotionPhrases = emotionPhrase[PHRASE_KEY];
	emotionPhrases.append(MIMIC_EMOTION_PHRASE);
	Json::Value& emotionBehaviors = emotionPhrase[BEHAVIOR_KEY];
	emotionBehaviors.append(INFORM_PHRASE_BEHAVIOR_1);

	Json::Value& promptFollowPhrase = val[MIMIC_PROMPT_FOLLOW_KEY];
	Json::Value& promptFollowPhrases = promptFollowPhrase[PHRASE_KEY];
	promptFollowPhrases.append(MIMIC_PROMPT_FOLLOW_PHRASE);
	Json::Value& promptFollowBehaviors = promptFollowPhrase[BEHAVIOR_KEY];
	promptFollowBehaviors.append(QUESTION_PHRASE_BEHAVIOR_1);

	Json::Value& promptCopyPhrase = val[MIMIC_PROMPT_COPY_KEY];
	Json::Value& promptCopyPhrases = promptCopyPhrase[PHRASE_KEY];
	promptCopyPhrases.append(MIMIC_PROMPT_COPY_PHRASE);
	Json::Value& promptCopyBehaviors = promptCopyPhrase[BEHAVIOR_KEY];
	promptCopyBehaviors.append(MIMIC_PROMPT_COPY_BEHAVIOR);

	Json::Value& correctPhrase = val[CORRECT_ANSWER_KEY];
	Json::Value& correctPhrases = correctPhrase[PHRASE_KEY];
	correctPhrases.append(MIMIC_CORRECT_PHRASE);
	Json::Value& correctBehaviors = correctPhrase[BEHAVIOR_KEY];
	correctBehaviors.append(CORRECT_ANSWER_BEHAVIOR_1);

	Json::Value& incorrectPhrase = val[INCORRECT_ANSWER_KEY];
	Json::Value& incorrectPhrases = incorrectPhrase[PHRASE_KEY];
	incorrectPhrases.append(MIMIC_INCORRECT_PHRASE);
	Json::Value& incorrectBehaviors = incorrectPhrase[BEHAVIOR_KEY];
	incorrectBehaviors.append(INCORRECT_ANSWER_BEHAVIOR_1);

	Json::Value& continueGameQuestionPhrase = val[CONTINUE_GAME_QUESTION_KEY];
	Json::Value& continueGameQuestionPhrases = continueGameQuestionPhrase[PHRASE_KEY];
	continueGameQuestionPhrases.append(MIMIC_CONTINUE_QUESTION_1);
	Json::Value& continueGameQuestionBehavior = continueGameQuestionPhrase[BEHAVIOR_KEY];
	continueGameQuestionBehavior.append(QUESTION_PHRASE_BEHAVIOR_1);

	Json::Value& finishPhrase = val[FINISH_KEY];
	Json::Value& finishPhrases = finishPhrase[PHRASE_KEY];
	finishPhrases.append(MIMIC_FINISH_PHRASE);
	Json::Value& finishBehavior = finishPhrase[BEHAVIOR_KEY];
	finishBehavior.append(INFORM_PHRASE_BEHAVIOR_1);

	return val;
}
