#include <NaoAutismWindow.h>

#include <QApplication>
#include <QPlastiqueStyle>

#include <ros/ros.h>

int main(int argc, char** argv)
{
	QApplication app(argc, argv);
	app.setStyle(new QPlastiqueStyle);

	//Create behavior objects
	QList<NaoBehavior>* behaviorList = new QList<NaoBehavior>;

	behaviorList->insert(0, NaoBehavior("Sit down", "sit_down"));

	//Create speech objects
	QList<NaoSpeech>* speechList = new QList<NaoSpeech>;

	speechList->insert(0, NaoSpeech("Well done", "Well done you are correct!"));

	//Init ros and create relevant objects
	ros::init(argc, argv, "nao_cntrl");

	NaoAutismWindow window(behaviorList, speechList);

	return app.exec();
}
