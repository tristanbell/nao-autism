/*
 * main.cpp
 *
 *  Created on: 29 Jul 2013
 *      Author: tristan
 */

#include <ros/ros.h>

#include <tf/tfMessage.h>
#include <geometry_msgs/TransformStamped.h>

#include <GameSettings.h>
#include <Phrase.h>

#include <iostream>

#include <string>
#include <vector>
#include <map>

#define NODE_NAME "emotion_game"
#define OPENNI_TRACKER "/openni_tracker"

#define HEAD_TF "head_"

bool tfReady;

bool checkRunningNodes(std::vector<std::string>&);
void checkTfTransforms();
void tfCallback(const tf::tfMessage);

int main(int argc, char** argv)
{
	ros::init(argc, argv, NODE_NAME);

	std::vector<std::string> node_names;
	ros::master::getNodes(node_names);

	//Construct settings object
	GameSettings settings;

	settings.setWait(2);
	settings.setTimeout(5);

	//For now, hardcode the Phrases
	std::map<std::string, Phrase> phraseMap;

	//Game name phrase(s)
	Phrase guess_start("Guess the emotion");
	phraseMap.insert(std::pair<std::string, Phrase>("Guess game start", guess_start));

	//Introduction phrase(s)
	Phrase guess_intro("The robot will do an emotion and you will have to guess what it is");
	phraseMap.insert(std::pair<std::string, Phrase>("Introduction", guess_intro));

	//Next emotion
	Phrase next_emotion("Next emotion");
	phraseMap.insert(std::pair<std::string, Phrase>("Next emotion", next_emotion));

	//Question phrase(s)
	Phrase guess_game_question("Was the robot % or %?");
	phraseMap.insert(std::pair<std::string, Phrase>("Question", guess_game_question));

	//Correct answer phrase(s)
	Phrase correct_answer_phrase("Well done, you guessed the robot was x");
	phraseMap.insert(std::pair<std::string, Phrase>("Generic correct", correct_answer_phrase));

	//Prompt phrase(s)
	Phrase generic_prompt_phrase("Try again");
	phraseMap.insert(std::pair<std::string, Phrase>("Generic prompt", generic_prompt_phrase));

	//Incorrect answer phrase(s)
	Phrase generic_incorrect_phrase("Lets try another emotion");
	phraseMap.insert(std::pair<std::string, Phrase>("Generic incorrect", generic_incorrect_phrase));

	//Game finished phrase(s)
	Phrase guess_finished("Guess the emotion is finished");
	phraseMap.insert(std::pair<std::string, Phrase>("Guess game finish", guess_finished));

	settings.setPhraseMap(phraseMap);

	while (!checkRunningNodes(node_names))
		sleep(1);

	//Check if the tf transform node is active and publishing
	checkTfTransforms();

	//All checks are done, start game
	while (ros::ok()){

		ros::spinOnce();
	}

	return 0;
}

void checkTfTransforms(){
	//Maybe do some kind of prompt here???

	//Create subscriber to the tf topic
	ros::NodeHandle handle;
	ros::Subscriber subscriber = handle.subscribe("/tf", 1, tfCallback);

	ros::Rate loopRate(60);

	ROS_INFO("Waiting for /tf topic publisher.");
	while (subscriber.getNumPublishers() == 0){
		if (!handle.ok()){
			ROS_INFO("Closing down.");

			exit(EXIT_FAILURE);
		}

		loopRate.sleep();
	}
	ROS_INFO("/tf topic publisher active.");

	ROS_INFO("Waiting for a head tf to be published.");
	while (!tfReady){
		ros::spinOnce();

		if (!handle.ok()){
			ROS_INFO("Closing down.");

			exit(EXIT_FAILURE);
		}

		loopRate.sleep();
	}

	ROS_INFO("Initialising done.");
}

void tfCallback(const tf::tfMessage msg)
{
	//Allow for application to continue when message containing tf for any head is recieved.
	geometry_msgs::TransformStamped transform = msg.transforms[0];
	if (transform.child_frame_id.find(HEAD_TF) != std::string::npos){
		ROS_INFO("/tf is now publishing, ready to go.");

		tfReady = true;
	}
}

bool checkRunningNodes(std::vector<std::string>& node_names)
{
	bool openniTrackerRunning = false;

	for (int i=0;i<node_names.size();i++){
		std::string& str = node_names[i];

		if (str == OPENNI_TRACKER){
			openniTrackerRunning = true;
		}
	}

	if (!openniTrackerRunning){
		std::cout << "OpenNI tracker node isn't running, attempting to start." << std::endl;

		std::system("rosrun openni_tracker openni_tracker &");

		std::cout << "Launched.";
	}else{
		return true;
	}

	return false;
}
