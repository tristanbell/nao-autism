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
#include <iostream>
#include <fstream>
#include <nao_autism_messages/Record.h>

const char* LOG_FILE_NAME = "timestamps2.log";

MimicGame::MimicGame(GameSettings& settings) : 	Game(settings),
												_userToTrack(0),
												_currentPoseClassification(-1)
{
	_performedBehavior = NULL;
	_currentState = INTRODUCTION;
	_performedEmotion = false;

	_classSubscriber = _nodeHandle.subscribe("/classification", 15, &MimicGame::classificationCallback, this);
	_recorderPublisher = _nodeHandle.advertise<nao_autism_messages::Record>("record", 10);
}

void MimicGame::startGame(void) {

	_currentState = INTRODUCTION;
	isDone = false;
	_userToTrack = 0;
	_emotionsPerformed = 0;
}

void MimicGame::perform(void) {
	switch (_currentState) {

		case INTRODUCTION: {
			introduction();
			_currentState = START_WAITING_TRACK;
			break;
		}

		case START_WAITING_TRACK: {
			//Prompt child to copy robot and perform required behavior
			std::vector<Phrase> phraseVector;
			if(_settings.getPhraseVector(MIMIC_PROMPT_COPY_KEY, phraseVector)){
				const Phrase& phr = sayAny(phraseVector);
				_naoControl.perform(phr.getRandomBehaviorName());

				sleep(_settings.getWait());
			}
			_currentState = WAITING_TRACK;

			break;
		}

		case WAITING_TRACK: {
			setUserToTrack();
			if (_userToTrack != 0) {
				_currentState = PERFORM_EMOTION;

				_naoControl.perform("init");
				sleep(_settings.getWait());
			}

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

			usleep(650000);
			_currentState = WAITING_MIMIC;

			break;
		}

		case WAITING_MIMIC: {
			// While in this state classificationCallback will run,
			// continuously updating _currentPoseClassification
			int desiredClassification = _performedBehavior->getClassification();

			// Correct pose
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

				_emotionsPerformed++;

				if (_emotionsPerformed >= _settings.getNumberOfEmotionsBeforeQuestion()){
					_currentState = ASK_QUESTION_CONTINUE;
					_emotionsPerformed = 0;
				}else{
					_currentState = PERFORM_EMOTION;
				}

				_poseQueue.clear();
				_timesPrompted = 0;

				break;
			}else{
				// Timeout for incorrect poses
				time_t currentTime;
				time(&currentTime);

				if (currentTime - _startWaitTime >= _settings.getTimeout()){
					_timesPrompted++;

					// Check to see if the number of prompts exceeds the max number.
					// If so, assume incorrect and perform another emotion.
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

						_poseQueue.clear();

						break;
					} else { // Prompt again
						std::vector<Phrase> phraseVector;
						if (_settings.getPhraseVector(PROMPT_KEY, phraseVector)){
							const Phrase& phrase = sayAny(phraseVector);

							if (phrase.getNumberOfBehaviors() != 0){
								std::string behavior = phrase.getRandomBehaviorName();

								_naoControl.perform(behavior);
							}
						}
						sleep(_settings.getWait());

						_naoControl.perform(_performedBehavior->getName());
						_currentState = PROMPT_MIMIC;
					}
				}
			}

			break;
		}

		case ASK_QUESTION_CONTINUE: {
			askToContinue();

			break;
		}

		case WAITING_ANSWER_CONTINUE: {
			Response response = waitToContinue();

			if (response == POSITIVE) {
				_userToTrack = 0;
				_currentState = STOP_SPEECH_RECOGNITION;
				_stateBuffer = START_WAITING_TRACK;
			}else if (response == NEGATIVE){
				_currentState = STOP_SPEECH_RECOGNITION;
				_stateBuffer = END_GAME;
			}

			break;
		}

		case START_SPEECH_RECOGNITION:{
			bool val = startSpeechRecognition();

			if (val){
				std::cout << "Successfully started speech recognition.\n";

				if (_stateBuffer != UNKNOWN){
					_currentState = _stateBuffer;
					_stateBuffer = UNKNOWN;
				}else{
					std::cout << "No state to go to from START_SPEECH_RECOGNITION.\n";
				}
			}else{
				std::cout << "Unable to start speech recognition, retrying.\n";
			}

			break;
		}

		case STOP_SPEECH_RECOGNITION:{
			bool val = stopSpeechRecognition();

			if (val){
				std::cout << "Successfully stopped speech recognition.\n";

				if (_stateBuffer != UNKNOWN){
					_currentState = _stateBuffer;
					_stateBuffer = UNKNOWN;
				}else{
					std::cout << "No state to go to from STOP_SPEECH_RECOGNITION.\n";
				}
			}else{
				std::cout << "Unable to stop speech recognition, retrying.\n";
			}

			break;
		}

		case END_GAME:{
			endGameSpeech();

			isDone = true;
			break;
		}

		default:{
			std::cout << "Unimplemented state: " << _currentState << std::endl;
			break;
		}
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
	std::list<nao_autism_messages::PoseClassification>::iterator queueIt = _poseQueue.begin();
	for (;queueIt != _poseQueue.end();queueIt++){
		//No need to check user number, it is checked in callback
		votes[(*queueIt).classification]++;
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
}

//Amount of frames to 'read' before classification is given
#define MAX_QUEUE_SIZE 15

/**
 * Receives PoseClassification messages, pushes them to a queue and
 * (depending on current state) calculates the most likely classification
 * from the queue or sets user to track.
 */
void MimicGame::classificationCallback(const nao_autism_messages::PoseClassification poseClass) {
	if (_currentState == WAITING_MIMIC) {
		//Ignore all users that aren't the current ones to track
		if (poseClass.user_number == _userToTrack){
			//Pose queue is continuous, so if it exceeds maximum size then pop front
			//before pushing
			if (_poseQueue.size() >= MAX_QUEUE_SIZE){
				_poseQueue.pop_front();
				_poseQueue.push_back(poseClass);

				setOverallClassification();
			}else{
				_poseQueue.push_back(poseClass);
			}
		}
	} else if (_currentState == WAITING_TRACK) {
		_poseQueue.push_back(poseClass);
	}
}

#define TRACK_POSE_DIST 0.6

/**
 * Assumes Nao is in a pose for the user to copy (hands on head).
 * Whichever user copies it correctly first is then the user to
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

		if (lDist < TRACK_POSE_DIST && rDist < TRACK_POSE_DIST) {
			_userToTrack = pc.user_number;
			printf("\n \nTracking user %d\n", _userToTrack);
			_poseQueue.clear();
			break;
		}
	}
}
