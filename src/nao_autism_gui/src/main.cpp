#include <nao_gui/NaoAutismWindow.h>
#include <NaoSpeechData.h>

#include <QApplication>
#include <QPlastiqueStyle>

#include <nao_control/NaoControl.h>

#include <ros/ros.h>

#include <tf/tfMessage.h>
#include <geometry_msgs/TransformStamped.h>
#include <nao_autism_messages/Record.h>

#include <recorder.h>

#include <vector>

#define NO_RECORD_FLAG "--no-record"
#define HELP_FLAG "--help"

using namespace std;

void init(const NaoSpeechData& data);
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

const string TIME_TO_PLAY_SPEECH  = "Time to play";
const string GUESS_THE_EMOTION_SPEECH = "Guess the emotion";

const string KEY = "hello";

int main(int argc, char** argv)
{
	//Init ros
	ros::init(argc, argv, "nao_cntrl");

	if (argc > 0){
		//Get filename from arguments
		char* fileName = argv[argc - 1];
		ROS_INFO("File name found: %s", fileName);

		//Check for no-record flag or help flag
		bool record = true;

		for (int i=0;i<argc;i++){
			std::string str(argv[i]);

			if (str == NO_RECORD_FLAG){
				ROS_INFO("--no-record flag found, not recording.");
				record = false;

				break;
			}else if (str == HELP_FLAG){
				std::cout << "To run nao_autism_gui you need to type the following:\n\trosrun nao_autism_gui run_autism_gui [flag] filename\n"
						<< "Where [flag] can be one of the following:\n\t--help: Displays how to run the GUI and what arguments can be given"
						<< " (when this flag is used, the GUI wont start).\n\t"
						<< "--no-record: Doesn't start the recording node, thus, no data is recorded.\n";

				return 0;
			}
		}

		ROS_INFO("Loading speech data");
		NaoSpeechData data = NaoSpeechData::load(fileName);

		//Prompt child to strike certain pose to initialise openni_tracker
		init(data);

		QApplication app(argc, argv);
		app.setStyle(new QPlastiqueStyle);

		vector<NaoBehavior> behaviors;

		NaoBehavior happy("Happy", HAPPY_BEHAVIOR);
		NaoBehavior sad("Sad", SAD_BEHAVIOR);
		NaoBehavior scared("Scared", SCARED_BEHAVIOR);
		NaoBehavior angry("Angry", ANGRY_BEHAVIOR);

		behaviors.push_back(happy);
		behaviors.push_back(sad);
		behaviors.push_back(scared);
		behaviors.push_back(angry);

		//No --no-record flag set, we shall record
	//	if (record){
			ROS_INFO("Starting to record data.");
			ros::NodeHandle nh;
			ros::Publisher pub = nh.advertise<nao_autism_messages::Record>("record", 10);

			ros::Rate rate(10);
			while (pub.getNumSubscribers() == 0){
				rate.sleep();
			}

			nao_autism_messages::Record msg;
			msg.record = true;
			pub.publish(msg);
	//	}


		//Init window and execute application
		nao_gui::NaoAutismWindow window(behaviors, data);

		return app.exec();
	}

	return 1;
}

void init(const NaoSpeechData& data)
{
	ROS_INFO("Initialising");

	using namespace nao_control;

	NaoControl control;

	//
	control.perform(STAND_UP_BEHAVIOR);

	//Prompt child to strike standard pose for calibrating openni_tracker
	control.say(data.get("INIT_POSE_PROMPT"));
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

	ROS_INFO("Waiting for a head tf to be published.");
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
	control.say(data.get("INIT_WELL_DONE"));
	control.perform(INIT_BEHAVIOR);

	//Say let's play
	control.say(data.get("INIT_PLAY"));
	control.perform("right_1");

	sleep(1);
}

void tfCallback(const tf::tfMessage msg)
{
	//Allow for application to continue when message containing tf for head_1 is recieved.
	geometry_msgs::TransformStamped transform = msg.transforms[0];
	if (transform.child_frame_id.find("head_") != std::string::npos){
		ROS_INFO("/tf is now publishing, ready to go.");

		ready = true;
	}
}
