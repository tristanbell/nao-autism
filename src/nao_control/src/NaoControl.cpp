#include <NaoControl.h>

const string NODE_NAME = "nao_control";

const string SPEECH_TOPIC_NAME = "speech";
const string BEHAVIOR_MANAGER_SERVER = "behavior_action";

/*
 * Sets up the required publishers/connections. This will wait until
 * everything is ready to use.
 *
 * Node: assumes that ros::init(int, char**, string) was already called.
 */
NaoControl::NaoControl() :
		nodeHandle(),
		behaviorTimeout(20.0),
		behaviorActionClient(BEHAVIOR_MANAGER_SERVER, true)
{
	speechPublisher = nodeHandle.advertise<std_msgs::String>(SPEECH_TOPIC_NAME, 1000);

	ROS_INFO("Waiting until speech publisher is ready...");
	ros::Rate loopRate(5);
	while (speechPublisher.getNumSubscribers() == 0){
		loopRate.sleep();
	}
	ROS_INFO("Speech publisher is ready.");

	ROS_INFO("Waiting for behavior manager to start.");
	behaviorActionClient.waitForServer();
	ROS_INFO("Connection made to behavior manager.");
}

/*
 * Speaks the following message defined by the string argument.
 */
void NaoControl::say(string message)
{
	std_msgs::String msg;
	msg.data = message;

	speechPublisher.publish(msg);
}

/*
 * Performs the following behavior defined by the string argument. It will
 * wait until the behavior has been fully executed.
 *
 * Returns whether the action has suceeded or not.
 */
bool NaoControl::perform(string behavior)
{
	WizOz::BehaviorGoal goal;

	goal.behavior_name = behavior;
	actionlib::SimpleClientGoalState state
			= behaviorActionClient.sendGoalAndWait(goal, behaviorTimeout);

	return state == state.SUCCEEDED;
}

//TESTING PURPOSES ONLY!
int main(int argc, char **argv)
{
	ros::init(argc, argv, NODE_NAME);

	NaoControl naoControl;

	naoControl.say("Hello! Here is an example.");

	return 0;
}
