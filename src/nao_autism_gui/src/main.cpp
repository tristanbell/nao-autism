#include <nao_gui/NaoAutismWindow.h>

#include <QApplication>
#include <QPlastiqueStyle>

#include <nao_control/NaoControl.h>

#include <ros/ros.h>

#include <tf/tfMessage.h>
#include <geometry_msgs/TransformStamped.h>

#include <recorder.h>

#include <vector>

using namespace std;

void init(void);
void tfCallback(const tf::tfMessage msg);

bool ready = false;

const string MIMIC_ME_1 = "Hey copy me";
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

	//Prompt child to strike certain pose to initialise openni_tracker
	init();

	QApplication app(argc, argv);
	app.setStyle(new QPlastiqueStyle);

	//Create relevant NaoSpeech and NaoBehavior objects for application
	NaoSpeech correctGeneric("Correct - Generic", "Well done! You guessed correctly.", "right_1");
	NaoSpeech tryAgainGeneric("Try again - Generic", "Try again", "wrong_1");
	NaoSpeech incorrectGeneric("Incorrect - Generic", "Lets try another", "wrong_1");

	vector<NaoBehavior> behaviors;

	vector<NaoSpeech> happySpeeches;

	happySpeeches.push_back(correctGeneric);
	happySpeeches.push_back(NaoSpeech("Correct", "Well done! You guessed I was happy!", "right_1"));
	happySpeeches.push_back(tryAgainGeneric);
	happySpeeches.push_back(incorrectGeneric);
	happySpeeches.push_back(NaoSpeech("Question", "Am I happy or sad?", "prompt_2"));
	happySpeeches.push_back(NaoSpeech("Question", "Am I happy or scared?", "prompt_2"));

	NaoBehavior happy("Happy", HAPPY_BEHAVIOR, happySpeeches);

	vector<NaoSpeech> sadSpeeches;

	sadSpeeches.push_back(correctGeneric);
	sadSpeeches.push_back(NaoSpeech("Correct", "Well done! You guessed I was sad!", "right_1"));
	sadSpeeches.push_back(tryAgainGeneric);
	sadSpeeches.push_back(incorrectGeneric);
	sadSpeeches.push_back(NaoSpeech("Question", "Am I sad or happy?", "prompt_2"));
	sadSpeeches.push_back(NaoSpeech("Question", "Am I sad or scared?", "prompt_2"));

	NaoBehavior sad("Sad", SAD_BEHAVIOR, sadSpeeches);

	vector<NaoSpeech> scaredSpeeches;

	scaredSpeeches.push_back(correctGeneric);
	scaredSpeeches.push_back(NaoSpeech("Correct", "Well done! You guessed I was scared!", "right_1"));
	scaredSpeeches.push_back(tryAgainGeneric);
	scaredSpeeches.push_back(incorrectGeneric);

	NaoBehavior scared("Scared", SCARED_BEHAVIOR, scaredSpeeches);

	behaviors.push_back(happy);
	behaviors.push_back(sad);
	behaviors.push_back(scared);

	//Init window and execute application
	nao_gui::NaoAutismWindow window(behaviors);

	//Everything is setup ok, start recording
	ROS_INFO("Starting to record data.");
//	Recorder::record("data");

	return app.exec();
}

void init()
{
	ROS_INFO("Initialising");

	using namespace nao_control;

	NaoControl control;

	//
	control.perform(STAND_UP_BEHAVIOR);

	//Prompt child to strike standard pose for calibrating openni_tracker
	control.say(MIMIC_ME_1);
	control.perform(ARMS_UP_BEHAVIOR);

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

	while (!ready){
		ros::spinOnce();

		if (!handle.ok()){
			ROS_INFO("Closing down.");

			exit(EXIT_FAILURE);
		}

		loopRate.sleep();
	}

	ROS_INFO("Initialising done.");

	//Congratulate child and go back to standard pose
	control.say(WELL_DONE_SPEECH);
	control.perform(INIT_BEHAVIOR);
}

void tfCallback(const tf::tfMessage msg)
{
	//Allow for application to continue when message containing tf for head_1 is recieved.
	geometry_msgs::TransformStamped transform = msg.transforms[0];
	if (transform.child_frame_id.find("head_1") != std::string::npos){
		ROS_INFO("/tf is now publishing, ready to go.");

		ready = true;
	}
}
