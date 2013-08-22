#include <BaseSettingsTab.h>

#include <QGridLayout>

const QString BaseSettingsTab::TAB_NAME = "Base settings";

void BaseSettingsTab::init()
{
	QGridLayout* layout = new QGridLayout;
	setLayout(layout);

	int rowNumber = 0;

	QLabel* waitLabel = new QLabel("Wait after speech (seconds):");
	layout->addWidget(waitLabel, rowNumber, 0);
	rowNumber++;

	_waitBox = new QSpinBox;
	_waitBox->setEnabled(false);
	layout->addWidget(_waitBox, rowNumber, 0);
	rowNumber++;

	QLabel* timeoutLabel = new QLabel("Wait before question timeout (seconds):");
	layout->addWidget(timeoutLabel, rowNumber, 0);
	rowNumber++;

	_timeoutBox = new QSpinBox;
	_timeoutBox->setEnabled(false);
	layout->addWidget(_timeoutBox, rowNumber, 0);
	rowNumber++;

	QLabel* triesLabel = new QLabel("Tries before failure:");
	layout->addWidget(triesLabel, rowNumber, 0);
	rowNumber++;

	_triesBox = new QSpinBox;
	_triesBox->setEnabled(false);
	layout->addWidget(_triesBox, rowNumber, 0);
	rowNumber++;

	QLabel* confidenceLabel = new QLabel("Speech recognition confidence (%):");
	layout->addWidget(confidenceLabel, rowNumber, 0);
	rowNumber++;

	_confidenceBox = new QSpinBox;
	_confidenceBox->setEnabled(false);
	_confidenceBox->setMaximum(100);
	layout->addWidget(_confidenceBox, rowNumber, 0);
	rowNumber++;
}

void BaseSettingsTab::onValueUpdated(int value)
{
	//Collect values and emit signal to model to ask for update
	BaseSettingsData data;

	data._timeout = _timeoutBox->value();
	data._tries = _triesBox->value();
	data._wait = _waitBox->value();
	data._confidence = _confidenceBox->value() / 100.0f;

	emit onSettingsUpdated(data);
}

#include <iostream>

void BaseSettingsTab::onSettingsLoaded(const BaseSettingsData& data)
{
	disconnectAll();

	_timeoutBox->setValue(data._timeout);
	_timeoutBox->setEnabled(true);

	_triesBox->setValue(data._tries);
	_triesBox->setEnabled(true);

	_waitBox->setValue(data._wait);
	_waitBox->setEnabled(true);

	_confidenceBox->setValue(data._confidence * 100);
	_confidenceBox->setEnabled(true);

	connectAll();
}

void BaseSettingsTab::connectAll()
{
	QObject::connect(_timeoutBox, SIGNAL(valueChanged(int)),
			this, SLOT(onValueUpdated(int)));
	QObject::connect(_triesBox, SIGNAL(valueChanged(int)),
			this, SLOT(onValueUpdated(int)));
	QObject::connect(_waitBox, SIGNAL(valueChanged(int)),
			this, SLOT(onValueUpdated(int)));
	QObject::connect(_confidenceBox, SIGNAL(valueChanged(int)),
			this, SLOT(onValueUpdated(int)));
}

void BaseSettingsTab::disconnectAll()
{
	QObject::disconnect(_timeoutBox, SIGNAL(valueChanged(int)),
			this, SLOT(onValueUpdated(int)));
	QObject::disconnect(_triesBox, SIGNAL(valueChanged(int)),
			this, SLOT(onValueUpdated(int)));
	QObject::disconnect(_waitBox, SIGNAL(valueChanged(int)),
			this, SLOT(onValueUpdated(int)));
	QObject::disconnect(_confidenceBox, SIGNAL(valueChanged(int)),
			this, SLOT(onValueUpdated(int)));
}
