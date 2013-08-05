/*
 * GuessGame.cpp
 *
 *  Created on: 29 Jul 2013
 *      Author: tristan
 */

#include <Game.h>
#include <GuessGame.h>

#include <Keys.h>

#define WORD_RECOGNIZED_TOPIC "word_recognized"
#define confidenceThreshold 0.25f // TODO: Implement in GameSettings class

#include <iostream>

GuessGame::GuessGame(GameSettings& settings) :  Game(settings),
											   _guessNodeHandle(),
											   _recognizedWords()
{
	_performedBehavior = NULL;

	_waitingSpeech = false;
	_currentState = INTRODUCTION;
	_performedEmotion = false;

	_speechSubscriber = _guessNodeHandle.subscribe(WORD_RECOGNIZED_TOPIC, 1000, &GuessGame::onSpeechRecognized, &*this);
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
				sayAny(phraseVector);
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

		std::vector<Phrase> questionVector;
		if (_settings.getPhraseVector(QUESTION_KEY, questionVector))
			sayAnyRandParts(questionVector, _performedBehavior->getActualName(), behaviorNames);

		_currentState = WAITING_ANSWER_QUESTION;

		//for testing...
		//_recognizedWords.push_back(std::pair<std::string, float>(_performedBehavior->getActualName(), 0));

		//Set the start time for waiting (used to calculate timeouts)
		time(&_startWaitTime);

		break;
	}

	case WAITING_ANSWER_QUESTION:{
		std::list<std::pair<std::string, float> >::iterator it = _recognizedWords.begin();

		//Wait until answer is given
		while (it != _recognizedWords.end()){
			std::pair<std::string, float>& pair = *it;

			//Check for correct answer
			if (pair.first == _performedBehavior->getActualName() && pair.second >= confidenceThreshold){
				std::list<std::string> parts;
				parts.push_back(_performedBehavior->getActualName());

				std::vector<Phrase> phraseVector;
				if (_settings.getPhraseVector(CORRECT_ANSWER_KEY, phraseVector))
					sayAny(phraseVector, parts);

				_recognizedWords.clear();

				//We shall ask the child here if they wish to continue with the current gamme
				_currentState = WAITING_ANSWER_CONTINUE;

				//for now, we shall go to the next emotion
				//_currentState = PERFORM_EMOTION;
				//_recognizedWords.clear();

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
				if (_settings.getPhraseVector(INCORRECT_ANSWER_KEY, phraseVector))
					sayAny(phraseVector, parts);

				_currentState = PERFORM_EMOTION;

				_recognizedWords.clear();
				_timesPrompted = 0;

				break;
			}else{
				std::vector<Phrase> phraseVector;
				if (_settings.getPhraseVector(PROMPT_KEY, phraseVector))
					sayAny(phraseVector);

				_currentState = ASK_QUESTION;
			}
		}

		break;
	}

	//TODO: Integrate this with the rest of the 'game'
	case WAITING_ANSWER_CONTINUE:{
		//Wait until yes/no is 'heard'
		std::list<std::pair<std::string, float> >::iterator it = _recognizedWords.begin();
		while (it != _recognizedWords.end()){
			std::pair<std::string, float>& pair = *it;

			if (pair.first == "yes" && pair.second >= confidenceThreshold){
				_currentState = PERFORM_EMOTION;

				_recognizedWords.clear();
				break;
			}else if (pair.first == "no" && pair.second >= confidenceThreshold){
				isDone = true;

				_recognizedWords.clear();

				break;
			}

			it++;
		}

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

void GuessGame::onSpeechRecognized(const nao_msgs::WordRecognized msg)
{
	//Check to see if speech is needed, if so push onto list
	if (!isDone && (_currentState == WAITING_ANSWER_CONTINUE || _currentState == WAITING_ANSWER_QUESTION)){
		for (int i=0;i<msg.words.size();i++){
			std::pair<std::string, float> pair(msg.words[i], msg.confidence_values[i]);

			std::cout << "Recognized word: " << msg.words[i] << ", confidence: " << msg.confidence_values[i] << "\n";

			_recognizedWords.push_back(pair);
		}
	}
}


