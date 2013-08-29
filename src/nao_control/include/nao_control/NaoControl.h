#ifndef NAOCONTROL_H_
#define NAOCONTROL_H_

#include <ros/ros.h>
#include <std_msgs/String.h>

#include <actionlib/client/terminal_state.h>
#include <actionlib/client/simple_action_client.h>

#include <nao_msgs/RunBehaviorAction.h>

#include <string>

namespace nao_control{

const std::string INIT_BEHAVIOR = "init";

class NaoControl{

public:
	NaoControl(bool loadSpeech = true);

	void say(const std::string&);
	void sayPreviousSpeech();
	const std::string getPreviousSpeech();

	bool perform(const std::string&);
	bool performPreviousBehavior();
	const std::string getPreviousBehavior();

	~NaoControl();

private:
	ros::NodeHandle nodeHandle;

	ros::Publisher speechPublisher;

	ros::Duration behaviorTimeout;
	actionlib::SimpleActionClient<nao_msgs::RunBehaviorAction> behaviorActionClient;

	std::string* previousBehavior;
	std::string* previousSpeech;

	bool performBehaviorInternal(const std::string& behavior)
	{
		nao_msgs::RunBehaviorGoal goal;

		std::cout << "[NaoControl] Performing behavior: " << behavior << std::endl;
		goal.behavior = behavior;
		actionlib::SimpleClientGoalState state
				= behaviorActionClient.sendGoalAndWait(goal, behaviorTimeout);

		return state == state.SUCCEEDED;
	}
};

}

#endif
