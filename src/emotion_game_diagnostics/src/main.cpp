#include <ros/ros.h>

#include <NodeBox.h>
#include <ExecutionControlBox.h>
#include <GeneralInformationBox.h>

#include <QApplication>
#include <QPlastiqueStyle>

#include <iostream>

#include <vector>
#include <string>

int main(int argc, char** argv)
{
	QApplication app(argc, argv);
	app.setStyle(new QPlastiqueStyle);

	ros::init(argc, argv, "nao_diagnostic_gui");

	//NodeBox* box = new NodeBox;
	//ExecutionControlBox* box = new ExecutionControlBox;
	GeneralInformationBox* box = new GeneralInformationBox;

	box->setVisible(true);

	return app.exec();
}
