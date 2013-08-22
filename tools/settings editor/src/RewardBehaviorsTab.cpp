#include <RewardBehaviorsTab.h>

#include <QGridLayout>

const QString RewardBehaviorsTab::TAB_NAME = "Reward behaviors";

void RewardBehaviorsTab::init()
{
	QGridLayout* layout = new QGridLayout;

	QLabel* listLabel = new QLabel("Reward behaviors:");
	layout->addWidget(listLabel, 0, 0);

	_behaviorList = new QListWidget;
	layout->addWidget(_behaviorList, 1, 0);

	QGridLayout* btnLayout = new QGridLayout;

	_addBehaviorBtn = new QPushButton("Add behavior");
	btnLayout->addWidget(_addBehaviorBtn, 0, 0);

	_editBehaviorBtn = new QPushButton("Edit behavior");
	btnLayout->addWidget(_editBehaviorBtn, 0, 1);

	_removeBehaviorBtn = new QPushButton("Remove behavior");
	btnLayout->addWidget(_removeBehaviorBtn, 0, 2);

	layout->addItem(btnLayout, 2, 0);

	setLayout(layout);
}
