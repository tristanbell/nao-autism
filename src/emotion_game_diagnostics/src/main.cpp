#include <ros/ros.h>

#include <NodeBox.h>
#include <ExecutionControlBox.h>
#include <GeneralInformationBox.h>

#include <QApplication>
#include <QPlastiqueStyle>

#include <iostream>

#include <vector>
#include <string>

#include <Window.h>

int main(int argc, char** argv)
{
	QApplication app(argc, argv);
	app.setStyle(new QPlastiqueStyle);

	ros::init(argc, argv, "nao_diagnostic_gui");

	Window* window = new Window;

	return app.exec();
}
