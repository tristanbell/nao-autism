#include <nao_control/NaoControl.h>

const std::string NODE_NAME = "nao_control";

const std::string SPEECH_TOPIC_NAME = "speech";
const std::string BEHAVIOR_MANAGER_SERVER = "behavior_action";

/*
 * Sets up the required publishers/connections. This will wait until
 * everything is ready to use.
 *
 * Node: assumes that ros::init(int, char**, string) was already called.
 */
nao_control::NaoControl::NaoControl() :
		nodeHandle(),
		behaviorTimeout(20.0),
		behaviorActionClient(BEHAVIOR_MANAGER_SERVER, true)
{
	previousSpeech = NULL;
	previousBehavior = NULL;

	speechPublisher = nodeHandle.advertise<std_msgs::String>(SPEECH_TOPIC_NAME, 1000);

	ROS_INFO("Waiting until speech publisher is ready...");
	ros::Rate loopRate(5);
	while (speechPublisher.getNumSubscribers() == 0){
		if (!nodeHandle.ok()){
			this->~NaoControl();

			return;
		}

		loopRate.sleep();
	}
	ROS_INFO("Speech publisher is ready.");

	ROS_INFO("Waiting for behavior manager to start.");
	while (!behaviorActionClient.isServerConnected()){
		if (!nodeHandle.ok()){
			this->~NaoControl();

			return;
		}

		loopRate.sleep();
	}

	ROS_INFO("Connection made to behavior manager.");
}

/*
 * Speaks the following message defined by the string argument.
 */
void nao_control::NaoControl::say(const std::string& message)
{
	std_msgs::String msg;
	msg.data = message;

	speechPublisher.publish(msg);

	if (previousSpeech != NULL)
		delete previousSpeech;

	previousSpeech = new std::string(message);
}

void nao_control::NaoControl::sayPreviousSpeech()
{
	if (previousSpeech != NULL){
		std::string speech(*previousSpeech);

		this->say(speech);
	}
}

const std::string nao_control::NaoControl::getPreviousSpeech()
{
	if (previousSpeech == NULL)
		return "";

	return *previousSpeech;
}

/*
 * Performs the following behavior defined by the string argument. It will
 * wait until the behavior has been fully executed.
 *
 * Returns whether the action has suceeded or not.
 */
bool nao_control::NaoControl::perform(const std::string& behavior)
{
	nao_control::BehaviorGoal goal;

	goal.behavior_name = behavior;
	actionlib::SimpleClientGoalState state
			= behaviorActionClient.sendGoalAndWait(goal, behaviorTimeout);

	if (previousBehavior != NULL)
		delete previousBehavior;

	previousBehavior = new std::string(behavior);

	return state == state.SUCCEEDED;
}

bool nao_control::NaoControl::performPreviousBehavior()
{
	if (previousBehavior == NULL)
		return false;

	std::string behavior(*previousBehavior);

	return this->perform(behavior);
}

const std::string nao_control::NaoControl::getPreviousBehavior()
{
	if (previousBehavior == NULL)
		return "";

	return *previousBehavior;
}

nao_control::NaoControl::~NaoControl()
{
	ros::shutdown();
}

//TESTING PURPOSES ONLY!
int main(int argc, char **argv)
{
	ros::init(argc, argv, NODE_NAME);

	nao_control::NaoControl naoControl;

	naoControl.say("Hello! Here is an example.");

	return 0;
}
