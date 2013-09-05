/*
 * AccuracyTesterView.h
 *
 *  Created on: Sep 4, 2013
 *      Author: parallels
 */

#ifndef ACCURACYTESTERVIEW_H_
#define ACCURACYTESTERVIEW_H_

#include <vector>

#include <QWidget>
#include <QPushButton>
#include <QComboBox>
#include <QLabel>
#include <QString>

#include <ros/ros.h>

#include <classification/accuracyTester/AccuracyTesterModel.h>

namespace classification
{

class AccuracyTesterView : public QWidget
{
	Q_OBJECT

public:
	AccuracyTesterView() : QWidget(), model()
	{
		learners.push_back("svm");
		learners.push_back("knn");
		learners.push_back("rdf");
		init();
	}

private:
	void init();

	/**
	 * This will run in a separate thread, getting accuracy and updating
	 * accordingly, as well as running ros::spin().
	 */
	void run();

	classification::AccuracyTesterModel model;
	vector<string> learners;

	QComboBox *classification_chooser;

	QLabel *svm_accuracy_label;
	QLabel *knn_accuracy_label;
	QLabel *rdf_accuracy_label;

	QPushButton *start_button;
	QPushButton *stop_button;
	QPushButton *reset_button;

	bool running;

Q_SIGNALS:
	void accuracyUpdated(map<string, float>);

private Q_SLOTS:
	void onChooserChanged(QString newClassification);
	void onAccuracyUpdated(map<string, float> accuracies);
	void onStartClicked();
	void onStopClicked();
	void onResetClicked();

}; // class AccuracyTesterView

}; // namespace classification


#endif /* ACCURACYTESTERVIEW_H_ */
