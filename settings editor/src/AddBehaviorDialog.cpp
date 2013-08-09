#include <AddBehaviorDialog.h>

#include <QGridLayout>
#include <QPushButton>
#include <QLabel>

const QString DIALOG_TITLE_NAME = "Add behavior";

void AddBehaviorDialog::init()
{
	QGridLayout* layout = new QGridLayout;
	setLayout(layout);

	QLabel* behaviorNameLabel = new QLabel("Behavior name");
	layout->addWidget(behaviorNameLabel, 0, 0);

	_behaviorName = new QLineEdit;
	layout->addWidget(_behaviorName, 1, 0);

	_createBtn = new QPushButton("Create");
	layout->addWidget(_createBtn, 2, 1);
}
