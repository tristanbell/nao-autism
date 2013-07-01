#ifndef NAOCONTROL_H_
#define NAOCONTROL_H_

#include <ros/ros.h>
#include <std_msgs/String.h>

#include <actionlib/client/terminal_state.h>
#include <actionlib/client/simple_action_client.h>

#include <nao_control/BehaviorAction.h>

#include <string>

namespace nao_control{

class NaoControl{

public:
	NaoControl();

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
	actionlib::SimpleActionClient<nao_control::BehaviorAction> behaviorActionClient;

	std::string* previousBehavior;
	std::string* previousSpeech;
};

}

#endif
