#include <BaseSettingsTab.h>

#include <QGridLayout>

const QString BaseSettingsTab::TAB_NAME = "Base settings";

void BaseSettingsTab::init()
{
	QGridLayout* layout = new QGridLayout;
	setLayout(layout);

	int rowNumber = 0;

	QLabel* waitLabel = new QLabel("Wait after speech:");
	layout->addWidget(waitLabel, rowNumber, 0);
	rowNumber++;

	_waitBox = new QLineEdit("5");
	layout->addWidget(_waitBox, rowNumber, 0);
	rowNumber++;

	QLabel* timeoutLabel = new QLabel("Wait before question timeout:");
	layout->addWidget(timeoutLabel, rowNumber, 0);
	rowNumber++;

	_timeoutBox = new QLineEdit("15");
	layout->addWidget(_timeoutBox, rowNumber, 0);
	rowNumber++;

	QLabel* triesLabel = new QLabel("Tries before failure:");
	layout->addWidget(triesLabel, rowNumber, 0);
	rowNumber++;

	_triesBox = new QLineEdit("3");
	layout->addWidget(_triesBox, rowNumber, 0);
	rowNumber++;
}
