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
#include <Phrase.h>
#include <Keys.h>

#include <string>
#include <vector>
#include <list>

#define WORD_RECOGNIZED_TOPIC "word_recognized"
#define SPEECH_TOPIC "recognizer/output"

class Game {
public:
	Game(GameSettings gs) : _settings(gs),
							_nodeHandle(),
							_naoControl(),
							_recognizedWords()
	{
		_speechSubscriber = _nodeHandle.subscribe(WORD_RECOGNIZED_TOPIC, 1000, &Game::onSpeechRecognized, this);
		_speechSubscriber2 = _nodeHandle.subscribe(SPEECH_TOPIC, 20, &Game::onSpeech, this);
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
		WAITING_TRACK,
		PERFORM_EMOTION,
		ASK_QUESTION,
		WAITING_ANSWER_QUESTION,
		ASK_QUESTION_CONTINUE,
		WAITING_ANSWER_CONTINUE,
		PROMPT_MIMIC,
		WAITING_MIMIC
	};

	// General settings/control
	nao_control::NaoControl _naoControl;
	GameSettings _settings;
	ros::NodeHandle _nodeHandle;
	ros::Subscriber _speechSubscriber;
	ros::Subscriber _speechSubscriber2;

	// Behaviors, speech
	State _currentState;
	bool _performedEmotion;
	Behavior* _performedBehavior;
	std::list<std::string> _recognizedWords;
	time_t _startWaitTime; // For timeouts
	int _timesPrompted;

	void introduction(void);
	void performEmotion(void);
	void askToContinue(void);
	void waitToContinue(void);

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
