#include "main_window.h"
#include <QtGui>
#include <iostream>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <std_msgs/Int32.h>

MainWindow::MainWindow(QWidget *parent) : QWidget(parent), bag("test.bag", rosbag::bagmode::Write)
{
	textArea = new QTextEdit;
	button1 = new QPushButton(tr("Button!"));
	x = 0;
	
	ros::Time::init();
	
	QObject::connect(button1, SIGNAL(clicked()), this, SLOT(test()));
	
	initUI();
}

/*
 * A test of a simple slot, which changes the text area and writes a
 * bagfile.
 */
void MainWindow::test(void)
{
	textArea->setPlainText(textArea->toPlainText() + tr("\nClick!"));
	
	if (x < 10)
		writeBagfile();
	else
		bag.close();
}

/*
 * Write a trivial bagfile of numbers, to test writing
 */
void MainWindow::writeBagfile()
{
	std_msgs::Int32 i;
	i.data = x++;
	bag.write("numbers", ros::Time::now(), i);
}

/*
 * Initialise window UI elements
 */
void MainWindow::initUI(void)
{
	resize(300, 300);
	QGridLayout *mainLayout = new QGridLayout;
	mainLayout->addWidget(textArea, 0, 0);
	mainLayout->addWidget(button1, 1, 0);
	
	setLayout(mainLayout);
	setWindowTitle(tr("The Great and Powerful Oz"));
}
