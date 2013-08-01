/*
 * GuessGame.cpp
 *
 *  Created on: 29 Jul 2013
 *      Author: tristan
 */

#include <Game.h>
#include <GuessGame.h>

#define WORD_RECOGNIZED_TOPIC "word_recognized"

#include <iostream>

GuessGame::GuessGame(GameSettings& settings) :  Game(settings),
											   _guessNodeHandle(),
											   _waitingSpeech(false),
											   _recognizedWords(),
											   _currentState(INTRODUCTION),
											   _performedEmotion(false)
{
	_speechSubscriber = _guessNodeHandle.subscribe("word_recognized", 1000, &GuessGame::onSpeechRecognized, &*this);
}

void GuessGame::startGame(void) {
	isDone = false;
	_waitingSpeech = false;

	//Reset state information
	_currentState = INTRODUCTION;
	_performedEmotion = false;
}

void GuessGame::perform(void) {
	switch (_currentState){

	case INTRODUCTION:{
		//Perform introduction speeches/actions, etc

		_currentState = PERFORM_EMOTION;

		break;
	}

	case PERFORM_EMOTION:{
		//Perform emotion and then ask the child a question
		if (_performedEmotion){

		}else{
			_performedEmotion = true;
		}

		_currentState = WAITING_ANSWER_QUESTION;

		break;
	}

	case WAITING_ANSWER_QUESTION:{
		std::list<std::pair<std::string, float> >::iterator it = _recognizedWords.begin();

		//Wait until answer is given
		while (it != _recognizedWords.end()){
			std::pair<std::string, float>& pair = *it;

			nao_control::NaoControl& cntrl = getNaoControl();

			//Check for correct answer
			if (pair.first == cntrl.getPreviousBehavior()){

				_currentState = PERFORM_EMOTION;
			}

			it++;
		}

		break;
	}

	case WAITING_ANSWER_CONTINUE:{
		std::list<std::pair<std::string, float> >::iterator it = _recognizedWords.begin();

		//Wait until yes/no is 'heard'
		while (it != _recognizedWords.end()){
			std::pair<std::string, float>& pair = *it;

			if (pair.first == "yes"){
				_currentState = PERFORM_EMOTION;

				_recognizedWords.clear();
				break;
			}else if (pair.first == "no"){
				isDone = false;

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
	_waitingSpeech = false;
}

void GuessGame::onSpeechRecognized(const nao_msgs::WordRecognized msg)
{
	std::cout << "Called!" << std::endl;

	//Check to see if speech is needed, if so push onto list
	if (!isDone && (_currentState == WAITING_ANSWER_CONTINUE || _currentState == WAITING_ANSWER_QUESTION)){
		for (int i=0;i<msg.words.size();i++){
			std::pair<std::string, float> pair(msg.words[i], msg.confidence_values[i]);

			std::cout << "Recognized word: " << msg.words[i] << ", confidence: " << msg.confidence_values[i] << "\n";

			_recognizedWords.push_back(pair);
		}
	}
}


