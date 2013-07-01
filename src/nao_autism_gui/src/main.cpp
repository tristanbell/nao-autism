#include <nao_gui/NaoAutismWindow.h>

#include <QApplication>
#include <QPlastiqueStyle>

#include <nao_control/NaoControl.h>

#include <ros/ros.h>

#include <vector>

using namespace std;

int main(int argc, char** argv)
{
	QApplication app(argc, argv);
	app.setStyle(new QPlastiqueStyle);

	vector<NaoBehavior> behaviors;

	vector<NaoSpeech> happySpeeches;

	happySpeeches.push_back(NaoSpeech("Correct - Generic", "Well done! You guessed correctly."));
	happySpeeches.push_back(NaoSpeech("Correct", "Well done! You guessed I was happy!"));

	NaoBehavior happy("Happy", "stand_up", happySpeeches);

	vector<NaoSpeech> sadSpeeches;

	sadSpeeches.push_back(NaoSpeech("Correct - Generic", "Well done! You guessed correctly."));
	sadSpeeches.push_back(NaoSpeech("Correct", "Well done! You guessed I was sad!"));

	NaoBehavior sad("Sad", "wipe_brow", sadSpeeches);

	behaviors.push_back(happy);
	behaviors.push_back(sad);

	//Init ros and create relevant objects
	ros::init(argc, argv, "nao_cntrl");

	nao_gui::NaoAutismWindow window(behaviors);

	return app.exec();
}
