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
}

void MimicGame::startGame(void) {
	_currentState = INTRODUCTION;
	isDone = false;
	_userToTrack = 0;
}

void MimicGame::perform(void) {
	switch (_currentState) {
		case INTRODUCTION: {
			introduction();
			_currentState = WAITING_TRACK;

			break;
		}

		case WAITING_TRACK: {
			setUserToTrack();
			if (_userToTrack != 0)
				_currentState = PERFORM_EMOTION;

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

			// Initialise time for mimic timeout
			time(&_startWaitTime);

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
				break;
			}

			// Timeout for incorrect poses
			time_t currentTime;
			time(&currentTime);

			if (currentTime - _startWaitTime >= _settings.getTimeout()){
				_timesPrompted++;

				//Check to see if the number of prompts exceeds the max number
				//if so, assume incorrect and perform another emotion
				if (_timesPrompted > _settings.getMaxPromptAmount()) {
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

					_timesPrompted = 0;

					break;
				} else {
					std::vector<Phrase> phraseVector;
					if (_settings.getPhraseVector(PROMPT_KEY, phraseVector)){
						const Phrase& phrase = sayAny(phraseVector);

						if (phrase.getNumberOfBehaviors() != 0){
							std::string behavior = phrase.getRandomBehaviorName();

							_naoControl.perform(behavior);
						}
					}
					sleep(_settings.getWait());

					_currentState = PROMPT_MIMIC;
				}
			}

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

		default:
			break;
	}

}


void MimicGame::endGame(void) {
	// Nothing to clean up
}

/**
 * Finds and sets the overall classification value from the queue of
 * PoseClassifications.
 */
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

/**
 * Receives PoseClassification messages, pushes them to a queue and
 * calculates the most likely classification from the queue.
 */
void MimicGame::classificationCallback(const nao_autism_messages::PoseClassification poseClass) {
	if (_currentState == WAITING_MIMIC) {
		if (_poseQueue.size() >= MAX_QUEUE_SIZE) {
			setOverallClassification();
		}

		_poseQueue.push_back(poseClass);
	}else if (_currentState == WAITING_TRACK) {
		_poseQueue.push_back(poseClass);
	}
}

#define TRACK_POSE_DIST 0.6

/**
 * Put the Nao in a pose for the user to copy (hands on head).
 * Whichever use copies it correctly first is then the user to
 * be tracked. All other users' classifications will be ignored
 * after this point.
 */
void MimicGame::setUserToTrack(void)
{
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

		//printf("Left: %f, Right: %f   \n", lDist, rDist);
//		std::flush(std::cout);

		if (lDist < TRACK_POSE_DIST && rDist < TRACK_POSE_DIST) {
			_userToTrack = pc.user_number;
			printf("\n \nTracking user %d\n", _userToTrack);
			_poseQueue.clear();
			break;
		}
	}
}































