/*
 * AccuracyTesterView.cpp
 *
 *  Created on: Sep 4, 2013
 *      Author: parallels
 */

#include <classification/accuracyTester/AccuracyTesterView.h>
#include <QGridLayout>
#include <QApplication>
#include <QPlastiqueStyle>
#include <boost/thread.hpp>

void classification::AccuracyTesterView::init() {
	QGridLayout* layout = new QGridLayout;

	QLabel *classification_label = new QLabel("Classification:");
	classification_chooser = new QComboBox();
	classification_chooser->addItem("Standing (0)");
	classification_chooser->addItem("Happy (1)");
	classification_chooser->addItem("Sad (2)");
	classification_chooser->addItem("Scared (3)");
	classification_chooser->addItem("Angry (4)");
	QObject::connect(classification_chooser, SIGNAL(activated(const QString&)),
				this, SLOT(onChooserChanged(QString)));

	QLabel *accuracy_label = new QLabel("Accuracy");
	svm_accuracy_label = new QLabel("SVM: ");
	knn_accuracy_label = new QLabel("KNN: ");
	rdf_accuracy_label = new QLabel("RDF: ");

	start_button = new QPushButton("Start");
	QObject::connect(start_button, SIGNAL(clicked()),
				this, SLOT(onStartClicked()));

	stop_button = new QPushButton("Stop");
	QObject::connect(stop_button, SIGNAL(clicked()),
				this, SLOT(onStopClicked()));

	reset_button = new QPushButton("Reset");
	QObject::connect(reset_button, SIGNAL(clicked()),
				this, SLOT(onResetClicked()));

	qRegisterMetaType<map<string, float> >("map<string, float>");
	QObject::connect(this, SIGNAL(accuracyUpdated(map<string, float>)), this,
			SLOT(onAccuracyUpdated(map<string, float>)));

	layout->addWidget(classification_label, 0, 0);
	layout->addWidget(classification_chooser, 0, 1);
	layout->addWidget(accuracy_label, 1, 1);
	layout->addWidget(svm_accuracy_label, 2, 0, 1, 3);
	layout->addWidget(rdf_accuracy_label, 3, 0, 1, 3);
	layout->addWidget(knn_accuracy_label, 4, 0, 1, 3);
	layout->addWidget(start_button, 5, 0);
	layout->addWidget(stop_button, 5, 1);
	layout->addWidget(reset_button, 5, 2);

	setLayout(layout);

	setBaseSize(100, 100);
	setVisible(true);

	boost::thread main(&classification::AccuracyTesterView::run, this);
}

void classification::AccuracyTesterView::run()
{
	ros::Rate loopRate(50);
	while (true) {
		if (running) {
			map<string, float> accuracies;
			for (int i = 0; i < learners.size(); i++) {
				float current_accuracy = model.getAccuracy(learners[i], model.getExpectedClass());
				accuracies[learners[i]] = current_accuracy;
			}
			emit accuracyUpdated(accuracies);
		}

		ros::spinOnce();
		loopRate.sleep();
	}
}

void classification::AccuracyTesterView::onChooserChanged(QString newClassification)
{
	int newClass = newClassification.toStdString().at(newClassification.size() - 2) - '0';

	model.setExpectedClass(newClass);

//	emit reset();
}

void classification::AccuracyTesterView::onAccuracyUpdated(map<string, float> accuracies)
{
	stringstream svmStr;

	svmStr << "SVM: " << setprecision(4) << accuracies["svm"] << "%";
	stringstream knnStr;
	knnStr << "KNN: " << setprecision(4) << accuracies["knn"] << "%";
	stringstream rdfStr;
	rdfStr << "RDF: " << setprecision(4) << accuracies["rdf"] << "%";
	svm_accuracy_label->setText(QString::fromStdString(svmStr.str()));
	knn_accuracy_label->setText(QString::fromStdString(knnStr.str()));
	rdf_accuracy_label->setText(QString::fromStdString(rdfStr.str()));
}

void classification::AccuracyTesterView::onStartClicked()
{
	running = true;
}

void classification::AccuracyTesterView::onStopClicked()
{
	running = false;
}

void classification::AccuracyTesterView::onResetClicked()
{
	svm_accuracy_label->setText("SVM: 0%");
	knn_accuracy_label->setText("KNN: 0%");
	rdf_accuracy_label->setText("RDF: 0%");
	model.resetAccuracy();
}

int main(int argc, char **argv) {
	QApplication app(argc, argv);
	app.setStyle(new QPlastiqueStyle);

	ros::init(argc, argv, "accuracy_tester");

	classification::AccuracyTesterView view;

	return app.exec();
}
