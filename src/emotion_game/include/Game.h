/*
 * Game.h
 *
 *  Created on: 29 Jul 2013
 *      Author: tristan
 */

#ifndef GAME_H_
#define GAME_H_

#include <ros/ros.h>
#include <GameSettings.h>
#include <nao_control/NaoControl.h>
#include <nao_msgs/WordRecognized.h>
#include <std_srvs/Empty.h>
#include <Phrase.h>
#include <Keys.h>

#include <string>
#include <vector>
#include <list>

#define WORD_RECOGNIZED_TOPIC "word_recognized"
#define SPEECH_TOPIC "recognizer/output"

#define START_NAO_SPEECH_RECOGNITION_NAME "nao_speech/start_recognition"
#define STOP_NAO_SPEECH_RECOGNITION_NAME "nao_speech/stop_recognition"
#define START_PS_SPEECH_RECOGNITION_NAME "recognizer/start"
#define STOP_PS_SPEECH_RECOGNITION_NAME "recognizer/stop"

class Game {
public:
	Game(GameSettings gs) : _settings(gs),
							_nodeHandle(),
							_naoControl(),
							_recognizedWords()
	{
		_naoSpeechSubscriber = _nodeHandle.subscribe(WORD_RECOGNIZED_TOPIC, 4, &Game::onSpeechRecognized, this);
		_startNaoSpeechService = _nodeHandle.serviceClient<std_srvs::Empty>(START_NAO_SPEECH_RECOGNITION_NAME);
		_stopNaoSpeechService = _nodeHandle.serviceClient<std_srvs::Empty>(STOP_NAO_SPEECH_RECOGNITION_NAME);

		_psSpeechSubscriber = _nodeHandle.subscribe(SPEECH_TOPIC, 4, &Game::onSpeech, this);
		_startPsSpeechService = _nodeHandle.serviceClient<std_srvs::Empty>(START_PS_SPEECH_RECOGNITION_NAME);
		_stopPsSpeechService = _nodeHandle.serviceClient<std_srvs::Empty>(STOP_PS_SPEECH_RECOGNITION_NAME);
	}

	virtual ~Game() { }

	GameSettings& getGameSettings()
	{
		return _settings;
	}

	nao_control::NaoControl& getNaoControl()
	{
		return _naoControl;
	}

	bool startSpeechRecognition();
	bool stopSpeechRecognition();

	virtual void startGame(void) = 0;
	virtual void perform(void) = 0;
	virtual void endGame(void) = 0;

	bool isDone;

protected:
	static const Phrase NULL_PHRASE;

	//State information related variables/structures
	enum State
	{
		INTRODUCTION,

		START_WAITING_TRACK,
		WAITING_TRACK,

		START_SPEECH_RECOGNITION,
		STOP_SPEECH_RECOGNITION,

		PERFORM_EMOTION,
		ASK_QUESTION,
		WAITING_ANSWER_QUESTION,
		ASK_QUESTION_CONTINUE,
		WAITING_ANSWER_CONTINUE,
		PROMPT_MIMIC,
		WAITING_MIMIC,
		END_GAME,
		UNKNOWN
	};

	enum Response
	{
		POSITIVE,
		NEGATIVE,
		NONE
	};

	// General settings/control
	nao_control::NaoControl _naoControl;
	GameSettings _settings;
	ros::NodeHandle _nodeHandle;

	ros::Subscriber _naoSpeechSubscriber;
	ros::ServiceClient _startNaoSpeechService;
	ros::ServiceClient _stopNaoSpeechService;

	ros::Subscriber _psSpeechSubscriber;
	ros::ServiceClient _startPsSpeechService;
	ros::ServiceClient _stopPsSpeechService;

	// Behaviors, speech
	State _currentState;
	State _stateBuffer; //Used for storing state required after a certain state has finished

	bool _performedEmotion;
	Behavior* _performedBehavior;
	std::list<std::string> _recognizedWords;
	time_t _startWaitTime; // For timeouts
	int _timesPrompted;
	int _emotionsPerformed;

	void introduction(void);
	void performEmotion(void);
	void askToContinue(void);
	Response waitToContinue(void);
	void endGameSpeech();

	void onSpeechRecognized(const nao_msgs::WordRecognized msg);
	void onSpeech(const std_msgs::String msg);

	const Phrase& say(const Phrase& phrase);
	const Phrase& say(const Phrase& phrase, const std::list<std::string> parts);
	const Phrase& sayRandParts(const Phrase& phrase, std::list<std::string> parts);
	const Phrase& sayRandParts(const Phrase& phrase, const std::string& required, std::list<std::string> parts);

	const Phrase& sayAny(const std::vector<Phrase>&);
	const Phrase& sayAny(const std::vector<Phrase>& phraseVector, const std::list<std::string>& parts);
	const Phrase& sayAnyRandParts(const std::vector<Phrase>& phraseVector, const std::list<std::string>& parts);
	const Phrase& sayAnyRandParts(const std::vector<Phrase>& phraseVector, const std::string& required, const std::list<std::string>& parts);
};


#endif /* GAME_H_ */
