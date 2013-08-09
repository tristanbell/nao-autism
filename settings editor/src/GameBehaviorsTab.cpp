#include <GameBehaviorsTab.h>

#include <QGridLayout>
#include <QPushButton>

const QString GameBehaviorsTab::TAB_NAME = "Game behaviors";

void GameBehaviorsTab::init()
{
	QGridLayout* layout = new QGridLayout;
	setLayout(layout);

	//Create widgets to display all of the 'actual' behaviors
	QLabel* behaviorNameLabel = new QLabel("Behavior names:");
	layout->addWidget(behaviorNameLabel, 0, 0);

	_behaviorNamesBox = new QComboBox;
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

	QString qBehaviorDialogTitle("Add behavior:");
	QString qBehaviorDialogLabel("Behavior name:");

	//Construct dialog for adding behaviors
	_addBehaviorDialog = new TextInputDialog(qBehaviorDialogTitle, qBehaviorDialogLabel);
	_addBehaviorDialog->setBaseSize(50, 50);
}

void GameBehaviorsTab::fillList(const BehaviorData& data)
{
	_availableBehaviorList->clear();
	std::list<std::string>::const_iterator it = data._behaviorNames.begin();
	while (it != data._behaviorNames.end()){
		QString qName = QString::fromStdString(*it);

		_availableBehaviorList->addItem(qName);

		it++;
	}
}

void GameBehaviorsTab::onBehaviorBoxIndexChanged(const QString& name)
{
	std::string behaviorName = name.toStdString();
	emit behaviorDataRequired(behaviorName);
}

void GameBehaviorsTab::onCreateBehaviorBtnClicked()
{
	_addBehaviorDialog->exec();

	if (_addBehaviorDialog->getResult() == TextInputDialog::CREATED){
		std::string key = _behaviorNamesBox->currentText().toStdString();
		std::string behaviorName = _addBehaviorDialog->getInput();

		//Add new behavior to list
		_availableBehaviorList->addItem(_addBehaviorDialog->getQInput());

		emit onBehaviorCreated(key, behaviorName);
	}
}

void GameBehaviorsTab::onBehaviorListLoaded(const std::list<BehaviorData>& data)
{
	QObject::disconnect(_behaviorNamesBox, SIGNAL(currentIndexChanged(const QString&)),
			this, SLOT(onBehaviorBoxIndexChanged(const QString&)));

	if (data.size() > 0){
		std::list<BehaviorData>::const_iterator it = data.begin();

		//Put first one into box and fill the list using it
		const BehaviorData& current = *it;
		QString qName = QString::fromStdString(current._actualName);
		_behaviorNamesBox->addItem(qName);
		fillList(current);
		it++;

		//Iterate through rest, adding to box
		while (it != data.end()){
			const BehaviorData& current = *it;
			qName = QString::fromStdString(current._actualName);

			_behaviorNamesBox->addItem(qName);
			it++;
		}
	}

	QObject::connect(_behaviorNamesBox, SIGNAL(currentIndexChanged(const QString&)),
			this, SLOT(onBehaviorBoxIndexChanged(const QString&)));
}

void GameBehaviorsTab::onBehaviorDataRetrieved(const BehaviorData& data)
{
	fillList(data);
}
