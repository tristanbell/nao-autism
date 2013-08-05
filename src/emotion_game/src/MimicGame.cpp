/*
 * MimicGame.cpp
 *
 *  Created on: 29 Jul 2013
 *      Author: tristan
 */

#include <Game.h>
#include <MimicGame.h>
#include <Keys.h>

MimicGame::MimicGame(GameSettings& settings) : Game(settings),
											   _mimicNodeHandle(),
											   _currentState(INTRODUCTION),
											   _performedEmotion(false)
{
	_performedBehavior = NULL;
}

void MimicGame::perform(void) {
	switch (_currentState) {
		case INTRODUCTION:
			introduction();
			break;

		case PERFORM_EMOTION:
			performEmotion();
			_currentState = PROMPT_MIMIC;
			break;

		case PROMPT_MIMIC:
			std::vector<Phrase> questionVector;

			std::list<std::string> parts;
			parts.push_back(_performedBehavior->getActualName());

			if (_settings.getPhraseVector(QUESTION_KEY, questionVector))
				sayAny(questionVector, parts);

			_currentState = WAITING_MIMIC;

			break;

		case WAITING_MIMIC:

			break;

		case WAITING_ANSWER_CONTINUE:

			break;

		default:
			break;
	}

}

void MimicGame::startGame(void) {

}

void MimicGame::endGame(void) {

}


