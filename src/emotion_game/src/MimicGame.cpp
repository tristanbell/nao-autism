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
											   _mimicNodeHandle()
{
	_performedBehavior = NULL;
	_currentState = INTRODUCTION;
	_performedEmotion = false;
	_classSubscriber = _mimicNodeHandle.subscribe("/classification", 15, &MimicGame::classificationCallback, this);
}

void MimicGame::startGame(void) {
	_currentState = INTRODUCTION;
}

void MimicGame::perform(void) {
	switch (_currentState) {
		case INTRODUCTION: {
			introduction();
			break;
		}

		case PERFORM_EMOTION: {
			performEmotion();
			_currentState = PROMPT_MIMIC;
			break;
		}

		case PROMPT_MIMIC: {
			std::vector<Phrase> questionVector;

			std::list<std::string> parts;
			parts.push_back(_performedBehavior->getActualName());

			if (_settings.getPhraseVector(QUESTION_KEY, questionVector))
				sayAny(questionVector, parts);

			_currentState = WAITING_MIMIC;

			break;
		}

		case WAITING_MIMIC: {
			// While in this state classificationCallback will run,
			// continuously updating _currentPoseClassification
			int desiredClassification = _performedBehavior->getClassification();

			if (_currentPoseClassification == desiredClassification) {
				// Say well done here, then ask to continue

				_currentState = WAITING_ANSWER_CONTINUE;
			}

			// Include a timeout for incorrect poses

			break;
		}

		case WAITING_ANSWER_CONTINUE: {

			break;
		}

		default: {
			break;
		}
	}

}

void MimicGame::endGame(void) {

}

void MimicGame::setOverallClassification(void) {
	std::map<short, int> votes;

	// Get the number of times each pose appears in _poseQueue
	for (int i = 0; i < _poseQueue.size(); i++) {
		votes[_poseQueue[i].classification]++;
	}

	// Get the pose with the highest number of votes
	std::map<short, int>::iterator it = votes.begin();
	std::pair<short, int> pair = *it;

	short highestPose = pair.first;
	int highestNum = pair.second;

	it++;

	while (it != votes.end()) {
		pair = *it;

		if (pair.second > highestNum) {
			highestPose = pair.first;
			highestNum = pair.second;
		}

		it++;
	}

	_currentPoseClassification = highestPose;

	_poseQueue.clear();
}

#define MAX_QUEUE_SIZE 15

void MimicGame::classificationCallback(const nao_autism_messages::PoseClassification poseClass) {
	if (_currentState == WAITING_MIMIC) {
		if (_poseQueue.size() >= MAX_QUEUE_SIZE) {
			setOverallClassification();
			std::cout << "Current class: " << _currentPoseClassification << "         \r";
		}

		_poseQueue.push_back(poseClass);
	}
}

































