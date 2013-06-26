#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QtGui>
#include <QWidget>

#include <rosbag/bag.h>

class MainWindow : public QWidget
{
	Q_OBJECT
	
public:
	MainWindow(QWidget *parent = 0);
	
private slots:	
	void test(void);
	
private:
	// Buttons, etc
	QPushButton *button1;
	QTextEdit *textArea;
	int x;
	rosbag::Bag bag;
	
	void initUI(void);
	void writeBagfile(void);
};

#endif
