/*
 * GuessGame.h
 *
 *  Created on: 30 Jul 2013
 *      Author: alex
 */

#ifndef GUESSGAME_H_
#define GUESSGAME_H_

#include <ros/ros.h>

#include <nao_msgs/WordRecognized.h>

#include <string>
#include <list>

class GuessGame : public Game {

public:
	GuessGame(GameSettings& settings);

	void perform(void);

protected:
	void startGame(void);
	void endGame(void);

private:
	ros::NodeHandle _guessNodeHandle;
	ros::Subscriber _speechSubscriber;

	bool _waitingSpeech;
	int currentWait;

	std::list<std::pair<std::string, float> > _recognizedWords;

	//State information related variables/structures
	enum State
	{
		INTRODUCTION,
		PERFORM_EMOTION,
		WAITING_ANSWER_QUESTION,
		WAITING_ANSWER_CONTINUE,
	};

	State _currentState;
	bool _performedEmotion;

	void onSpeechRecognized(const nao_msgs::WordRecognized msg);

};

#endif /* GUESSGAME_H_ */
