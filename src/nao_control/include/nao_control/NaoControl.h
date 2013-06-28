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

	void say(const std::string);
	bool perform(const std::string);

private:
	ros::NodeHandle nodeHandle;

	ros::Publisher speechPublisher;

	ros::Duration behaviorTimeout;
	actionlib::SimpleActionClient<nao_control::BehaviorAction> behaviorActionClient;
};

}

#endif