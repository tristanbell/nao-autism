#include <GeneralInformationBox.h>

#include <QGridLayout>

#include <sstream>

void GeneralInformationBox::init()
{
	setTitle("General");

	_nodeHandle = ros::NodeHandle();
	_diagnosticSubscriber = _nodeHandle.subscribe("/diagnostics", 1, &GeneralInformationBox::diagnosticsCallback, this);

	QGridLayout* layout = new QGridLayout;

	QLabel* batteryStatusLbl = new QLabel("Battery level: ");
	layout->addWidget(batteryStatusLbl, 0, 0);

	_batteryLevelLabel = new QLabel("Unknown");
	layout->addWidget(_batteryLevelLabel, 0, 1);

	QLabel* jointTempLabel = new QLabel("Average joint temperature: ");
	layout->addWidget(jointTempLabel, 1, 0);

	_averageJointTemperatureLabel = new QLabel("Unknown");
	layout->addWidget(_averageJointTemperatureLabel, 1, 1);

	QLabel* emptyWidget = new QLabel;
	layout->addWidget(emptyWidget, 2, 0);

	QLabel* stopBatteryLabel = new QLabel("Stop when battery level is less than: ");
	layout->addWidget(stopBatteryLabel, 3, 0);

	_batteryLevelSpinner = new QSpinBox();
	_batteryLevelSpinner->setMinimum(10);
	_batteryLevelSpinner->setMaximum(100);
	_batteryLevelSpinner->setSuffix("%");
	layout->addWidget(_batteryLevelSpinner, 3, 1);
	QObject::connect(_batteryLevelSpinner, SIGNAL(valueChanged(int)),
			this, SLOT(batteryLevelSpinnerChanged(int)));

	QLabel* stopTempLabel = new QLabel("Stop when average joint temperature is greater than: ");
	layout->addWidget(stopTempLabel, 4, 0);

	_averageTempSpinner = new QSpinBox();
	_averageTempSpinner->setMinimum(30);
	_averageTempSpinner->setMaximum(70);
	QObject::connect(_averageTempSpinner, SIGNAL(valueChanged(int)),
			this, SLOT(averageTemperatureSpinnerChanged(int)));

	std::stringstream degreesSuffix;
	degreesSuffix << "\xB0" << "c";
	QString qDegreeSuffix = QString::fromStdString(degreesSuffix.str());

	_averageTempSpinner->setSuffix(qDegreeSuffix);
	layout->addWidget(_averageTempSpinner, 4, 1);

	setLayout(layout);

	_diagnosticCheckerThread = boost::thread(&GeneralInformationBox::run, this);

	QObject::connect(this, SIGNAL(requestUpdate()),
			this, SLOT(onUpdate()));
}

void GeneralInformationBox::run()
{
	ros::Rate loopRate(60);
	while (ros::ok()){
		loopRate.sleep();
		ros::spinOnce();
	}
}

void GeneralInformationBox::diagnosticsCallback(const diagnostic_msgs::DiagnosticArray& msg)
{
	emit requestUpdate();

	_naoDiagnostics.diagnosticMessageCallback(msg);
}

void GeneralInformationBox::batteryLevelSpinnerChanged(int value)
{
	_naoDiagnostics.setBatteryLevelLimit(value);
}

void GeneralInformationBox::averageTemperatureSpinnerChanged(int value)
{
	_naoDiagnostics.setJointTemperatureLimit(value);
}

void GeneralInformationBox::onUpdate()
{
	std::stringstream batteryLevelLimit;
	batteryLevelLimit << _naoDiagnostics.getBatteryLevelLimit() << "%";
	QString qBatteryLevel = QString::fromStdString(batteryLevelLimit.str());

	std::stringstream jointLevelLimit;
	jointLevelLimit << _naoDiagnostics.getAverageJointTemperature() << "\xB0" << "c";
	QString qAvgJointTemp = QString::fromStdString(jointLevelLimit.str());

	_batteryLevelLabel->setText(qBatteryLevel);
	_averageJointTemperatureLabel->setText(qAvgJointTemp);
}
