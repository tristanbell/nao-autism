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

#include <iostream>

GuessGame::GuessGame(GameSettings& settings) :  Game(settings),
											   _guessNodeHandle(),
											   _waitingSpeech(false),
											   _recognizedWords(),
											   _currentState(INTRODUCTION),
											   _performedEmotion(false)
{
	_performedBehavior = NULL;

	_speechSubscriber = _guessNodeHandle.subscribe("word_recognized", 1000, &GuessGame::onSpeechRecognized, &*this);
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
		//std::cout << "State: INTRODUCTION" << std::endl;

		//Perform start and instruction phrases/actions, etc
		std::vector<Phrase> phraseVector;
		if (getGameSettings().getPhraseVector(START_KEY, phraseVector))
			sayAny(phraseVector);

		sleep(_settings.getWait());

		if (getGameSettings().getPhraseVector(INSTRUCTION_KEY, phraseVector))
			sayAny(phraseVector);

		_currentState = PERFORM_EMOTION;

		break;
	}

	case PERFORM_EMOTION:{
		//std::cout << "State: PERFORM_EMOTION" << std::endl;

		//Inform child that a new emotion is being performed
		if (_performedEmotion){
			std::vector<Phrase> phraseVector;
			if (getGameSettings().getPhraseVector(GUESS_NEXT_KEY, phraseVector)){
				sayAny(phraseVector);
			}
		}else{
			_performedEmotion = true;
		}

		const std::vector<Behavior>& behaviorVector = _settings.getBehaviorVector();

		//Find a new random behavior to perform
		int index = 0;
		while (true){
			index = rand() % behaviorVector.size();
			const Behavior& ref = behaviorVector[index];

			if (_performedBehavior == NULL || ref.getActualName() != _performedBehavior->getActualName())
				break;
		}

		const Behavior& ref = behaviorVector[index];
		_naoControl.perform(ref.getName());

		if (_performedBehavior != NULL)
			delete _performedBehavior;

		_performedBehavior = new Behavior(ref);

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
			if (pair.first == _performedBehavior->getActualName()){
				std::list<std::string> parts;
				parts.push_back(_performedBehavior->getActualName());

				std::vector<Phrase> phraseVector;
				if (_settings.getPhraseVector(CORRECT_ANSWER_KEY, phraseVector))
					sayAny(phraseVector, parts);

				//We shall ask the child here if they wish to continue with the current gamme

				//for now, we shall go to the next emotion
				_currentState = PERFORM_EMOTION;
				_recognizedWords.clear();

				_timesPrompted = 0;

				break;
			}

			it++;
		}

		//Check to see if the duration of time has exceeded the maximum wait
		//if so, prompt the child and ask the question again.
		time_t currentTime;
		time(&currentTime);

		if (currentTime - _startWaitTime >= _settings.getTimeout()){
			std::vector<Phrase> phraseVector;
			if (_settings.getPhraseVector(PROMPT_KEY, phraseVector))
				sayAny(phraseVector);

			_timesPrompted++;

			_currentState = ASK_QUESTION;
		}

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
		}

		break;
	}

	case WAITING_ANSWER_CONTINUE:{
		//std::cout << "State: WAITING_ANSWER_CONTINUE" << std::endl;

		std::list<std::pair<std::string, float> >::iterator it = _recognizedWords.begin();

		//Wait until yes/no is 'heard'
		while (it != _recognizedWords.end()){
			std::pair<std::string, float>& pair = *it;

			if (pair.first == "yes"){
				isDone = true;

				_recognizedWords.clear();
				break;
			}else if (pair.first == "no"){
				_recognizedWords.clear();

				break;
			}

			it++;
		}
		break;
	}

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


