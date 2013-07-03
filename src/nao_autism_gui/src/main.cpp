#include <nao_gui/NaoAutismWindow.h>

#include <QApplication>
#include <QPlastiqueStyle>

#include <nao_control/NaoControl.h>

#include <ros/ros.h>

#include <tf/tfMessage.h>
#include <geometry_msgs/TransformStamped.h>

#include <vector>

using namespace std;

void init(void);
void tfCallback(const tf::tfMessage msg);

bool ready = false;

const string MIMIC_ME_1 = "Mimic me";
const string WELL_DONE_SPEECH = "Well done";

QString HAPPY_BEHAVIOR = "happy_1";
QString SAD_BEHAVIOR = "sad_1";

QString SCARED_BEHAVIOR = "scared_1";
QString SUPRISED_BEHAVIOR = "suprise_1";
QString ANGRY_BEHAVIOR = "angry_1";

const string STAND_UP_BEHAVIOR = "stand_up";
const string ARMS_UP_BEHAVIOR = "arms_up";

int main(int argc, char** argv)
{
	//Init ros
	ros::init(argc, argv, "nao_cntrl");

	init();

	QApplication app(argc, argv);
	app.setStyle(new QPlastiqueStyle);

	vector<NaoBehavior> behaviors;

	vector<NaoSpeech> happySpeeches;

	happySpeeches.push_back(NaoSpeech("Correct - Generic", "Well done! You guessed correctly."));
	happySpeeches.push_back(NaoSpeech("Correct", "Well done! You guessed I was happy!"));

	NaoBehavior happy("Happy", HAPPY_BEHAVIOR, happySpeeches);

	vector<NaoSpeech> sadSpeeches;

	sadSpeeches.push_back(NaoSpeech("Correct - Generic", "Well done! You guessed correctly."));
	sadSpeeches.push_back(NaoSpeech("Correct", "Well done! You guessed I was sad!"));

	NaoBehavior sad("Sad", SAD_BEHAVIOR, sadSpeeches);

	behaviors.push_back(happy);
	behaviors.push_back(sad);

	nao_gui::NaoAutismWindow window(behaviors);

	return app.exec();
}

void init()
{
	ROS_INFO("Initialising");

	using namespace nao_control;

	NaoControl control;

	control.perform(STAND_UP_BEHAVIOR);

	//Prompt
	control.say(MIMIC_ME_1);
	control.perform(ARMS_UP_BEHAVIOR);

	//Check for tf messages
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

	while (!ready){
		ros::spinOnce();

		if (!handle.ok()){
			ROS_INFO("Closing down.");

			exit(EXIT_FAILURE);
		}

		loopRate.sleep();
	}

	ROS_INFO("Initialising done.");

	control.say(WELL_DONE_SPEECH);
	control.perform(INIT_BEHAVIOR);
}

void tfCallback(const tf::tfMessage msg)
{
	geometry_msgs::TransformStamped transform = msg.transforms[0];
	if (transform.child_frame_id.find("head_1") != std::string::npos){
		ROS_INFO("/tf is now publishing, ready to go.");

		ready = true;
	}
}
