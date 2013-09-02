/*
 * MimicGame.h
 *
 *  Created on: 30 Jul 2013
 *      Author: alex
 */

#ifndef MIMICGAME_H_
#define MIMICGAME_H_

#include <Game.h>
#include <ros/ros.h>

#include <nao_msgs/WordRecognized.h>
#include <nao_autism_messages/PoseClassification.h>
#include <geometry_msgs/TransformStamped.h>
#include <Phrase.h>

#include <string>
#include <vector>
#include <list>

class MimicGame : public Game {

public:
	MimicGame(GameSettings&);

	void startGame(void);
	void perform(void);
	void endGame(void);

private:
	int _userToTrack;
	ros::Subscriber _classSubscriber;
	std::vector<nao_autism_messages::PoseClassification> _poseQueue;
	int _currentPoseClassification;

	ros::Publisher rec_pub;

	void setOverallClassification(void);
	void classificationCallback(const nao_autism_messages::PoseClassification poseClass);
	void setUserToTrack(void);
	void writeToLogBehavior(void);
	void writeToLogPrompt(void);
	void writeToLogAnswer(const bool& answer);
};

#endif /* MIMICGAME_H_ */
