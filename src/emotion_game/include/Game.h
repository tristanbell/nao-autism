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

class Game {
public:
	Game(GameSettings gs) : _settings(gs),
							_nodeHandle(),
							_naoControl(),
							_recognizedWords()
	{
		std::cout << "Initialising game..." << std::endl;
		_speechSubscriber = _nodeHandle.subscribe(WORD_RECOGNIZED_TOPIC, 1000, &Game::onSpeechRecognized, this);
		std::cout << "Initialising done, subscribed to speech." << std::endl;
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
		PERFORM_EMOTION,
		ASK_QUESTION,
		WAITING_ANSWER_QUESTION,
		ASK_QUESTION_CONTINUE,
		WAITING_ANSWER_CONTINUE,
		PROMPT_MIMIC,
		WAITING_MIMIC
	};

	nao_control::NaoControl _naoControl;
	GameSettings _settings;
	ros::NodeHandle _nodeHandle;
	ros::Subscriber _speechSubscriber;

	State _currentState;
	bool _performedEmotion;
	Behavior* _performedBehavior;
	std::list<std::pair<std::string, float> > _recognizedWords;

	void introduction(void);
	void performEmotion(void);
	void askToContinue(void);
	void waitToContinue(void);

	bool startSpeechRecognition();
	bool stopSpeechRecognition();
	void onSpeechRecognized(const nao_msgs::WordRecognized msg);

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
