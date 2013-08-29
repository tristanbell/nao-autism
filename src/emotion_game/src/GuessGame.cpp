/*
 * GuessGame.cpp
 *
 *  Created on: 29 Jul 2013
 *      Author: tristan
 */

#include <Game.h>
#include <GuessGame.h>

#include <Keys.h>

#include <iostream>

GuessGame::GuessGame(GameSettings& settings) :  Game(settings)
{
	_performedBehavior = NULL;

	_currentState = INTRODUCTION;
	_performedEmotion = false;
}

void GuessGame::startGame(void) {
	//Reset state information
	_currentState = INTRODUCTION;

	isDone = false;
	_performedEmotion = false;
	_timesPrompted = 0;
	_emotionsPerformed = 0;
}

void GuessGame::perform(void) {
	switch (_currentState){

	case INTRODUCTION:{
		introduction();
		break;
	}

	case PERFORM_EMOTION:{
		if (_emotionsPerformed < _settings.getNumberOfEmotionsBeforeQuestion()){
			//Inform child that a new emotion is being performed
			if (_performedEmotion){
				std::vector<Phrase> phraseVector;
				if (getGameSettings().getPhraseVector(GUESS_NEXT_KEY, phraseVector)){
					const Phrase& phrase = sayAny(phraseVector);

					if (phrase.getNumberOfBehaviors() != 0){
						std::string behavior = phrase.getRandomBehaviorName();

						_naoControl.perform(behavior);
					}
				}
			}else{
				_performedEmotion = true;
			}

			performEmotion();
			_emotionsPerformed++;

			_currentState = ASK_QUESTION;
		}else{
			_emotionsPerformed = 0;
			_currentState = ASK_QUESTION_CONTINUE;
		}

		break;
	}

	case ASK_QUESTION:{
		askQuestion();

		//for testing...
		//_recognizedWords.push_back(std::pair<std::string, float>(_performedBehavior->getActualName(), 0));

		//Set the start time for waiting (used to calculate timeouts)
		startSpeechRecognition();
		time(&_startWaitTime);

		_recognizedWords.clear();

		break;
	}

	case WAITING_ANSWER_QUESTION:{
		std::list<std::string>::iterator it = _recognizedWords.begin();

		//Wait until answer is given
		while (it != _recognizedWords.end()){
			std::string& word = *it;

			//Check for correct answer
			if (word == _performedBehavior->getActualName()){
				handleCorrectAnswer();
				return;
			}

//			}else if (checkIncorrectAnswer(word)){ //Check for incorrect answer, if found then break out of loop
//				handleIncorrectAnswer();
//
//				break;
//			}

			it++;
		}

		//Check to see if the duration of time has exceeded the maximum wait
		//if so, prompt the child and ask the question again.
		time_t currentTime;
		time(&currentTime);

		if (currentTime - _startWaitTime >= _settings.getTimeout()){
			_timesPrompted++;

			//Check to see if the number of prompts exceeds the max number
			//if so, assume incorrect and perform another emotion
			if (_timesPrompted > _settings.getMaxPromptAmount()){
				handleTimeout();

				break;
			}else{
				promptChild();
			}
		}

		break;
	}

	case ASK_QUESTION_CONTINUE:{
		askToContinue();
		_recognizedWords.clear();

		break;
	}

	case WAITING_ANSWER_CONTINUE:{
		waitToContinue();

		break;
	}

	case END_GAME:{
		endGameSpeech();

		isDone = true;
		break;
	}

	default:
		break;

	}
}

void GuessGame::endGame(void) {
	//Clean up
	if (_performedBehavior == NULL)
		delete _performedBehavior;
}

void GuessGame::askQuestion()
{
	//Collect all other behavior names into string
	std::list<std::string> behaviorNames;

	//Collect the other behaviors
	const std::vector<Behavior>& behaviorVector = _settings.getBehaviorVector();
	for (int i=0;i<behaviorVector.size();i++){
		const std::string& current = behaviorVector[i].getActualName();

		if (current != _performedBehavior->getActualName()){
			behaviorNames.push_back(current);
		}
	}
	sleep(_settings.getWait());

	std::vector<Phrase> questionVector;
	if (_settings.getPhraseVector(QUESTION_KEY, questionVector)){
		const Phrase& phrase = sayAnyRandParts(questionVector, _performedBehavior->getActualName(), behaviorNames);

		if (phrase.getNumberOfBehaviors() != 0){
			std::string behavior = phrase.getRandomBehaviorName();

			_naoControl.perform(behavior);
		}
	}

	_currentState = WAITING_ANSWER_QUESTION;
}

void GuessGame::handleCorrectAnswer()
{
	stopSpeechRecognition();

	std::list<std::string> parts;
	parts.push_back(_performedBehavior->getActualName());

	std::vector<Phrase> phraseVector;
	if (_settings.getPhraseVector(CORRECT_ANSWER_KEY, phraseVector)){
		const Phrase& phrase = sayAny(phraseVector, parts);

		if (phrase.getNumberOfBehaviors() != 0){
			std::string behavior = phrase.getRandomBehaviorName();

			_naoControl.perform(behavior);
		}
	}
	sleep(_settings.getWait());

	_recognizedWords.clear();

	//We shall ask the child here if they wish to continue with the current game
	_currentState = PERFORM_EMOTION;

	_timesPrompted = 0;
}

bool GuessGame::checkIncorrectAnswer(const std::string& answer)
{
	stopSpeechRecognition();

	for (int i=0;i<_settings.getBehaviorVector().size();i++){
		const Behavior& current = _settings.getBehaviorVector()[i];

		//If there was an incorrect guess, then we prompt child again
		if (current.getActualName() == answer){
			return true;
		}
	}

	return false;
}

void GuessGame::handleIncorrectAnswer()
{
	_timesPrompted++;
	promptChild();
}

void GuessGame::handleTimeout()
{
	stopSpeechRecognition();

	//Collect last performed behaviors name, as incorrect phrase may require it
	std::list<std::string> parts;
	parts.push_back(_performedBehavior->getActualName());

	//Alert child
	std::vector<Phrase> phraseVector;
	if (_settings.getPhraseVector(INCORRECT_ANSWER_KEY, phraseVector)){
		const Phrase& phrase = sayAny(phraseVector, parts);

		if (phrase.getNumberOfBehaviors() != 0){
			std::string behavior = phrase.getRandomBehaviorName();

			_naoControl.perform(behavior);
		}
	}
	sleep(_settings.getWait());

	_currentState = PERFORM_EMOTION;

	_recognizedWords.clear();
	_timesPrompted = 0;
}

void GuessGame::promptChild()
{
	stopSpeechRecognition();

	std::vector<Phrase> phraseVector;
	if (_settings.getPhraseVector(PROMPT_KEY, phraseVector)){
		const Phrase& phrase = sayAny(phraseVector);

		if (phrase.getNumberOfBehaviors() != 0){
			std::string behavior = phrase.getRandomBehaviorName();

			_naoControl.perform(behavior);
		}
	}
	sleep(_settings.getWait());

	_currentState = ASK_QUESTION;
}


