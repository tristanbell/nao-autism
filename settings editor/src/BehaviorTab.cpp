#include <BehaviorTab.h>

#include <QGridLayout>
#include <QPushButton>

const QString BehaviorTab::TAB_NAME = "Behaviors";

void BehaviorTab::init()
{
	QGridLayout* layout = new QGridLayout;
	setLayout(layout);

	//Create widgets to display all of the 'actual' behaviors
	QLabel* behaviorNameLabel = new QLabel("Behavior names:");
	layout->addWidget(behaviorNameLabel, 0, 0);

	QComboBox* _behaviorNamesBox = new QComboBox;
	layout->addWidget(_behaviorNamesBox, 1, 0);

	//Create widgets to display current behaviors (where each name refers to a behavior on the Nao)
	QLabel* availableBehaviorLabel = new QLabel("Available behaviors:");
	layout->addWidget(availableBehaviorLabel, 2, 0);

	_availableBehaviorList = new QListWidget;
	layout->addWidget(_availableBehaviorList, 3, 0);

	QGridLayout* addRemoveBtnLayout = new QGridLayout;

	_addBtn = new QPushButton("Add behavior");
	addRemoveBtnLayout->addWidget(_addBtn, 0, 0);

	QObject::connect(_addBtn, SIGNAL(clicked()), this, SLOT(onCreateBehaviorBtnClicked()));

	_removeBtn = new QPushButton("Remove behavior");
	addRemoveBtnLayout->addWidget(_removeBtn, 0, 1);

	layout->addLayout(addRemoveBtnLayout, 4, 0);

	//Construct dialog for adding behaviors
	_addBehaviorDialog = new AddBehaviorDialog;
	_addBehaviorDialog->setBaseSize(50, 50);
}

void BehaviorTab::onCreateBehaviorBtnClicked()
{
	int rtn = _addBehaviorDialog->exec();
}
