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

	_waitingSpeech = false;
	_currentState = INTRODUCTION;
	_performedEmotion = false;
}

void GuessGame::startGame(void) {
	//Reset state information
	_currentState = INTRODUCTION;

	isDone = false;
	_waitingSpeech = false;
	_performedEmotion = false;
	_timesPrompted = 0;
}

void GuessGame::perform(void) {
	switch (_currentState){

	case INTRODUCTION:{
		introduction();
		break;
	}

	case PERFORM_EMOTION:{
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
		_currentState = ASK_QUESTION;
		break;
	}

	case ASK_QUESTION:{
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

		//for testing...
		//_recognizedWords.push_back(std::pair<std::string, float>(_performedBehavior->getActualName(), 0));

		//Set the start time for waiting (used to calculate timeouts)
		startSpeechRecognition();
		time(&_startWaitTime);

		break;
	}

	case WAITING_ANSWER_QUESTION:{
		std::list<std::pair<std::string, float> >::iterator it = _recognizedWords.begin();

		//Wait until answer is given
		while (it != _recognizedWords.end()){
			std::pair<std::string, float>& pair = *it;

			//Check for correct answer
			if (pair.first == _performedBehavior->getActualName() && pair.second >= _settings.getConfidenceThreshold()){
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
				_currentState = ASK_QUESTION_CONTINUE;

				_timesPrompted = 0;

				return;
			}

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
				stopSpeechRecognition();

				_recognizedWords.clear();
				_timesPrompted = 0;

				break;
			}else{
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
		}

		break;
	}

	case ASK_QUESTION_CONTINUE:{
		askToContinue();
		break;
	}

	case WAITING_ANSWER_CONTINUE:{
		waitToContinue();
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


