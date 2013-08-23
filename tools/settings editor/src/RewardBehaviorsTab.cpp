#include <RewardBehaviorsTab.h>

#include <QGridLayout>
#include <QMessageBox>

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
	_addBehaviorBtn->setEnabled(false);
	btnLayout->addWidget(_addBehaviorBtn, 0, 0);

	_editBehaviorBtn = new QPushButton("Edit behavior");
	_editBehaviorBtn->setEnabled(false);
	btnLayout->addWidget(_editBehaviorBtn, 0, 1);

	_removeBehaviorBtn = new QPushButton("Remove behavior");
	_removeBehaviorBtn->setEnabled(false);
	btnLayout->addWidget(_removeBehaviorBtn, 0, 2);

	layout->addItem(btnLayout, 2, 0);

	setLayout(layout);

	QString qTitle = "";
	QString qLabelName = "Behavior name:";
	_behaviorDialog = new TextInputDialog(qTitle, qLabelName);

	QObject::connect(_addBehaviorBtn, SIGNAL(clicked()),
			this, SLOT(onAddBehaviorBtnClicked()));
	QObject::connect(_editBehaviorBtn, SIGNAL(clicked()),
			this, SLOT(onEditBehaviorBtnClicked()));
	QObject::connect(_removeBehaviorBtn, SIGNAL(clicked()),
			this, SLOT(onRemoveBehaviorBtnClicked()));
}

void RewardBehaviorsTab::onBehaviorListLoaded(const std::list<std::string>& list)
{
	QObject::disconnect(_behaviorList, SIGNAL(itemSelectionChanged()),
			this, SLOT(onBehaviorListItemChanged()));

	_behaviorList->clear();
	_addBehaviorBtn->setEnabled(true);

	std::list<std::string>::const_iterator it = list.begin();
	while (it != list.end()){
		const std::string& current = *it;

		_behaviorList->addItem(QString::fromStdString(current));

		it++;
	}

	QObject::connect(_behaviorList, SIGNAL(itemSelectionChanged()),
			this, SLOT(onBehaviorListItemChanged()));
}

void RewardBehaviorsTab::onAddBehaviorBtnClicked()
{
	QString title = "Add behavior";
	_behaviorDialog->setTitle(title);

	_behaviorDialog->exec();

	if (_behaviorDialog->getResult() == TextInputDialog::CREATED){
		QString qBehaviorName = _behaviorDialog->getQInput();

		if (qBehaviorName != ""){
			std::string behaviorName = qBehaviorName.toStdString();

			_behaviorList->addItem(qBehaviorName);

			emit addBehavior(behaviorName);
		}else{
			QMessageBox::information(this, "Error", "Unable to add behavior as nothing was entered.");
		}
	}
}

void RewardBehaviorsTab::onEditBehaviorBtnClicked()
{
	QString title = "Edit behavior";
	_behaviorDialog->setTitle(title);

//	std::string key = _phraseGroupBox->currentText().toStdString();
//	std::string phrase = _phrasesList->currentItem()->text().toStdString();
//
//	QListWidgetItem* item = _phrasesList->takeItem(_phrasesList->currentIndex().row());

	QString name = _behaviorList->currentItem()->text();
	_behaviorDialog->setInput(name);

	_behaviorDialog->exec();

	if (_behaviorDialog->getResult() == TextInputDialog::CREATED){
		QString qBehaviorName = _behaviorDialog->getQInput();

		if (qBehaviorName != ""){
			std::string behaviorName = qBehaviorName.toStdString();

			_behaviorList->currentItem()->setText(qBehaviorName);

			std::string oldBehaviorName = name.toStdString();

			emit removeBehavior(oldBehaviorName);
			emit addBehavior(behaviorName);
		}else{
			QMessageBox::information(this, "Error", "Unable to edit behavior as nothing was entered.");
		}
	}
}

void RewardBehaviorsTab::onRemoveBehaviorBtnClicked()
{
	QMessageBox::StandardButton btn = QMessageBox::question(this, "Delete behavior", "Are you sure you want to delete this behavior?"
			,QMessageBox::Yes | QMessageBox::No);

	if (btn == QMessageBox::Yes){
		std::string behaviorName = _behaviorList->currentItem()->text().toStdString();

		_behaviorList->removeItemWidget(_behaviorList->currentItem());

		emit removeBehavior(behaviorName);
	}
}

void RewardBehaviorsTab::onBehaviorListItemChanged()
{
	_addBehaviorBtn->setEnabled(true);
	_editBehaviorBtn->setEnabled(true);
	_removeBehaviorBtn->setEnabled(true);
}
