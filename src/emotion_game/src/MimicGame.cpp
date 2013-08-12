/*
 * MimicGame.cpp
 *
 *  Created on: 29 Jul 2013
 *      Author: tristan
 */

#include <Game.h>
#include <MimicGame.h>
#include <Keys.h>

MimicGame::MimicGame(GameSettings& settings) : Game(settings)
{
	_performedBehavior = NULL;
	_currentState = INTRODUCTION;
	_performedEmotion = false;
	_classSubscriber = _nodeHandle.subscribe("/classification", 15, &MimicGame::classificationCallback, this);
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

			if (_settings.getPhraseVector(MIMIC_PROMPT_FOLLOW_KEY, questionVector))
				sayAny(questionVector);
			std::cout << "Finished prompt." << std::endl;

			_currentState = WAITING_MIMIC;

			break;
		}

		case WAITING_MIMIC: {
			// While in this state classificationCallback will run,
			// continuously updating _currentPoseClassification
			int desiredClassification = _performedBehavior->getClassification();

			if (_currentPoseClassification == desiredClassification) {
				// Say well done
				std::list<std::string> parts;
				parts.push_back(_performedBehavior->getActualName());

				std::vector<Phrase> phraseVector;
				if (_settings.getPhraseVector(CORRECT_ANSWER_KEY, phraseVector)) {
					const Phrase& phrase = sayAny(phraseVector, parts);

					if (phrase.getNumberOfBehaviors() != 0) {
						std::string behavior = phrase.getRandomBehaviorName();

						_naoControl.perform(behavior);
					}
				}
				sleep(_settings.getWait());

				_currentState = ASK_QUESTION_CONTINUE;
			}

			// Include a timeout for incorrect poses

			break;
		}

		case ASK_QUESTION_CONTINUE: {
			askToContinue();
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
//			std::cout << "Current class: " << _currentPoseClassification << "         \r";
//			std::flush(std::cout);
		}

		_poseQueue.push_back(poseClass);
	}
}

































