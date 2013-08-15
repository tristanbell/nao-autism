/*
 * MimicGame.cpp
 *
 *  Created on: 29 Jul 2013
 *      Author: tristan
 */

#include <Game.h>
#include <MimicGame.h>
#include <Keys.h>
#include <boost/foreach.hpp>

MimicGame::MimicGame(GameSettings& settings) : Game(settings)
{
	_performedBehavior = NULL;
	_currentState = INTRODUCTION;
	_performedEmotion = false;
	_classSubscriber = _nodeHandle.subscribe("/classification", 15, &MimicGame::classificationCallback, this);
	_userToTrack = 0;
}

void MimicGame::startGame(void) {
	std::cout << "Mimic game starting..." << std::endl;
	_currentState = INTRODUCTION;
}

void MimicGame::perform(void) {
	switch (_currentState) {
		case INTRODUCTION: {
			introduction();
			while (_userToTrack == 0)
				setUserToTrack();
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
			waitToContinue();
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
	// Map of PoseClassification's classification mapped to the number
	// of times that classification appears in the poseQueue.
	std::map<int, int> votes;

	// Get the number of times each pose appears in _poseQueue
	for (int i = 0; i < _poseQueue.size(); i++) {
		// Only pay attention to current user
		if (_poseQueue[i].user_number == _userToTrack) {
			votes[_poseQueue[i].classification]++;
		}
	}

	// Get the pose with the highest number of votes
	std::map<int, int>::iterator it = votes.begin();
	std::pair<int, int> pair = *it;

	int highestPose = pair.first;
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
		}

		_poseQueue.push_back(poseClass);
	}
}

#define TRACK_POSE_DIST 0.7

/**
 * Put the Nao in a pose for the user to copy (hands on head).
 * Whichever use copies it correctly first is then the user to
 * be tracked. All other users' classifications will be ignored
 * after this point.
 */
void MimicGame::setUserToTrack(void)
{
	std::vector<Phrase> phraseVector;
	if (_settings.getPhraseVector(CORRECT_ANSWER_KEY, phraseVector)) {
		const Phrase& phrase = sayAny(phraseVector);

		if (phrase.getNumberOfBehaviors() != 0) {
			std::string behavior = phrase.getRandomBehaviorName();

			_naoControl.perform(behavior);
		}
	}

	BOOST_FOREACH(nao_autism_messages::PoseClassification pc, _poseQueue) {
		geometry_msgs::Vector3 head = pc.pose_data[0].transform.translation;
		geometry_msgs::Vector3 left_hand = pc.pose_data[5].transform.translation;
		geometry_msgs::Vector3 right_hand = pc.pose_data[8].transform.translation;

		float ldx = head.x - left_hand.x;
		float ldy = head.y - left_hand.y;
		float ldz = head.z - left_hand.z;
		float rdx = head.x - right_hand.x;
		float rdy = head.y - right_hand.y;
		float rdz = head.z - right_hand.z;

		float lDist = sqrt(ldx*ldx + ldy*ldy + ldz*ldz);
		float rDist = sqrt(rdx*rdx + rdy*rdy + rdz*rdz);

		if (lDist < TRACK_POSE_DIST && rDist < TRACK_POSE_DIST) {
			_userToTrack = pc.user_number;
			break;
		}
	}
}































