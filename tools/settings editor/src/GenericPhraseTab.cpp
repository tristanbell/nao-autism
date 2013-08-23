#include <GenericPhraseTab.h>

#include <QGridLayout>
#include <QMessageBox>
#include <QLabel>
#include <QString>

#include <iostream>

const QString GenericPhraseTab::TAB_NAME = "General phrases";

QString GenericPhraseTab::getTabName() const
{
	return _tabName;
}

void GenericPhraseTab::init()
{
	QGridLayout* layout = new QGridLayout;
	setLayout(layout);

	//Create combobox for phrase 'groups'
	QLabel* phraseGroupLabel = new QLabel("Phrase groups:");
	layout->addWidget(phraseGroupLabel, 0, 0);

	_phraseGroupBox = new QComboBox;
	layout->addWidget(_phraseGroupBox, 1, 0);

	//Create main phrase layout
	QGridLayout* phraseLayout = new QGridLayout;

	QLabel* phrasesLabel = new QLabel("Phrases:");
	phraseLayout->addWidget(phrasesLabel, 0, 0);

	_phrasesList = new QListWidget;
	phraseLayout->addWidget(_phrasesList, 1, 0);

	//Create phrase add/remove button layout
	QGridLayout* phraseBtnLayout = new QGridLayout;

	_addPhraseBtn = new QPushButton("Add phrase");
	_addPhraseBtn->setEnabled(false);
	phraseBtnLayout->addWidget(_addPhraseBtn, 0, 0);

	_editPhraseBtn = new QPushButton("Edit phrase");
	_editPhraseBtn->setEnabled(false);
	phraseBtnLayout->addWidget(_editPhraseBtn, 0, 1);

	_removePhraseBtn = new QPushButton("Remove phrase");
	_removePhraseBtn->setEnabled(false);
	phraseBtnLayout->addWidget(_removePhraseBtn, 0, 2);

	//Add the button layout to the main phrase layout
	phraseLayout->addLayout(phraseBtnLayout, 2, 0);

	//Add main phrase layout to the main layout of the given tab
	layout->addLayout(phraseLayout, 2, 0);

	//Create layout for adding behaviors
	QGridLayout* behaviorLayout = new QGridLayout;

	QLabel* behaviorLabel = new QLabel("Behaviors (these will be performed when the robot is speaking):");
	behaviorLayout->addWidget(behaviorLabel, 0, 0);

	_behaviorList = new QListWidget;
	behaviorLayout->addWidget(_behaviorList, 1, 0);

	//Create behavior add/remove button layout
	QGridLayout* behaviorBtnLayout = new QGridLayout;

	_addBehaviorBtn = new QPushButton("Add behavior");
	_addBehaviorBtn->setEnabled(false);
	behaviorBtnLayout->addWidget(_addBehaviorBtn, 0, 0);

	_editBehaviorBtn = new QPushButton("Edit behavior");
	_editBehaviorBtn->setEnabled(false);
	behaviorBtnLayout->addWidget(_editBehaviorBtn, 0, 1);

	_removeBehaviorBtn = new QPushButton("Remove behavior");
	_removeBehaviorBtn->setEnabled(false);
	behaviorBtnLayout->addWidget(_removeBehaviorBtn, 0, 2);

	//Add behavior add/remove button layout to the main behavior layout
	behaviorLayout->addLayout(behaviorBtnLayout, 2, 0);

	//Add main behavior layout to the main layout
	layout->addLayout(behaviorLayout, 3, 0);

	//Create instance of input dialog
	_inputDialog = new TextInputDialog;

	//Connect signals to required slots
	QObject::connect(_phrasesList, SIGNAL(itemSelectionChanged()),
			this, SLOT(onPhraseListItemChanged()));

	QObject::connect(_addPhraseBtn, SIGNAL(clicked()),
			this, SLOT(addPhraseButtonClicked()));
	QObject::connect(_editPhraseBtn, SIGNAL(clicked()),
			this, SLOT(editPhraseButtonClicked()));
	QObject::connect(_removePhraseBtn, SIGNAL(clicked()),
			this, SLOT(removePhraseButtonClicked()));

	QObject::connect(_behaviorList, SIGNAL(itemSelectionChanged()),
			this, SLOT(onBehaviorListItemChanged()));

	QObject::connect(_addBehaviorBtn, SIGNAL(clicked()),
			this, SLOT(addBehaviorButtonClicked()));
	QObject::connect(_editBehaviorBtn, SIGNAL(clicked()),
			this, SLOT(editBehaviorButtonClicked()));
	QObject::connect(_removeBehaviorBtn, SIGNAL(clicked()),
			this, SLOT(removeBehaviorButtonClicked()));
}

void GenericPhraseTab::setPhraseGroup(const std::map<std::string, PhraseGroupData>& phraseGroups)
{
	//Stop request for data when we are loading as we don't require it
	QObject::disconnect(_phraseGroupBox, SIGNAL(currentIndexChanged(const QString&)),
			this, SLOT(phraseGroupBoxIndexChanged(const QString&)));

	//Clear old phrases
	_phraseGroupBox->clear();

	if (phraseGroups.size() > 0){
		//Insert new phrases into box
		std::map<std::string, PhraseGroupData>::const_iterator it = phraseGroups.begin();

		const std::pair<std::string, PhraseGroupData>& pair = *it;
		loadIntoLists(pair.second); // Load data into the list for given 'key'
		QString qName = QString::fromStdString(pair.first);
		_phraseGroupBox->addItem(qName);
		it++;

		while (it != phraseGroups.end()){
			const std::pair<std::string, PhraseGroupData>& pair = *it;

			QString qName = QString::fromStdString(pair.first);
			_phraseGroupBox->addItem(qName);

			it++;
		}
	}

	//Reconnect combobox to allow for requests when index is changed
	QObject::connect(_phraseGroupBox, SIGNAL(currentIndexChanged(const QString&)),
			this, SLOT(phraseGroupBoxIndexChanged(const QString&)));
}

void GenericPhraseTab::setCurrentPhraseGroup(const PhraseGroupData& data)
{
	loadIntoLists(data);
}

void GenericPhraseTab::onPhraseGroupLoaded(const std::map<std::string, PhraseGroupData>& group)
{
	setPhraseGroup(group);

	_addPhraseBtn->setEnabled(true);
	_editPhraseBtn->setEnabled(false);
	_removePhraseBtn->setEnabled(false);

	_addBehaviorBtn->setEnabled(true);
	_editBehaviorBtn->setEnabled(false);
	_removeBehaviorBtn->setEnabled(false);
}

void GenericPhraseTab::onPhraseGroupRetrieved(const PhraseGroupData& data)
{
	setCurrentPhraseGroup(data);

	_editPhraseBtn->setEnabled(false);
	_editBehaviorBtn->setEnabled(false);
	_removePhraseBtn->setEnabled(false);
	_removeBehaviorBtn->setEnabled(false);
}

void GenericPhraseTab::loadIntoLists(const PhraseGroupData& data)
{
	_phrasesList->clear();
	_behaviorList->clear();

	//Load phrases
	if (data.phraseVector.size() > 0){
		std::list<std::string>::const_iterator it = data.phraseVector.begin();
		while (it != data.phraseVector.end()){
			const std::string& current = *it;

			QString qString = QString::fromStdString(current);
			_phrasesList->addItem(qString);

			it++;
		}
	}

	//Load behaviors
	if (data.behaviorVector.size() > 0){
		std::list<std::string>::const_iterator it = data.behaviorVector.begin();
		while (it != data.behaviorVector.end()){
			const std::string& current = *it;

			QString qString = QString::fromStdString(current);
			_behaviorList->addItem(qString);

			it++;
		}
	}
}

void GenericPhraseTab::phraseGroupBoxIndexChanged(const QString& text)
{
	std::string key = text.toStdString();
	emit onPhraseGroupRequired(key);
}

void GenericPhraseTab::onPhraseListItemChanged()
{
	_editPhraseBtn->setEnabled(true);
	_removePhraseBtn->setEnabled(true);
}

void GenericPhraseTab::addPhraseButtonClicked()
{
	QString title = "Add phrase";
	_inputDialog->setTitle(title);

	QString lbl = "Phrase name:";
	_inputDialog->setLabelName(lbl);

	_inputDialog->exec();

	if (_inputDialog->getResult() == TextInputDialog::CREATED){
		std::string phrase = _inputDialog->getInput();

		if (phrase != ""){
			QString qKey = _phraseGroupBox->currentText();
			std::string key = qKey.toStdString();

			//Add to the phrase list
			_phrasesList->addItem(_inputDialog->getQInput());

			emit onPhraseCreated(key, phrase);
		}else{
			QMessageBox::information(this, "Error", "Unable to add phrase as nothing was entered.");
		}
	}
}

void GenericPhraseTab::editPhraseButtonClicked()
{
	QString title = "Edit phrase";
	_inputDialog->setTitle(title);

	QString lbl = "Phrase name:";
	_inputDialog->setLabelName(lbl);

	QString input = _phrasesList->currentItem()->text();
	_inputDialog->setInput(input);

	_inputDialog->exec();

	if (_inputDialog->getResult() == TextInputDialog::CREATED){
		std::string phrase = _inputDialog->getInput();

		if (phrase != ""){
			QString qKey = _phraseGroupBox->currentText();
			std::string key = qKey.toStdString();

			std::string oldPhrase = _phrasesList->currentItem()->text().toStdString();
			_phrasesList->currentItem()->setText(_inputDialog->getQInput());

			emit onPhraseRemoved(key, oldPhrase);
			emit onPhraseCreated(key, phrase);
		}else{
			QMessageBox::information(this, "Error", "Unable to edit phrase as nothing was entered.");
		}
	}
}

void GenericPhraseTab::removePhraseButtonClicked()
{
	QMessageBox::StandardButton btn = QMessageBox::question(this, "Delete phrase", "Are you sure you want to delete this phrase?"
			,QMessageBox::Yes | QMessageBox::No);

	if (btn == QMessageBox::Yes){
		std::string key = _phraseGroupBox->currentText().toStdString();
		std::string phrase = _phrasesList->currentItem()->text().toStdString();

		QListWidgetItem* item = _phrasesList->takeItem(_phrasesList->currentIndex().row());

		emit onPhraseRemoved(key, phrase);
	}
}

void GenericPhraseTab::onBehaviorListItemChanged()
{
	_editBehaviorBtn->setEnabled(true);
	_removeBehaviorBtn->setEnabled(true);
}

void GenericPhraseTab::addBehaviorButtonClicked()
{
	QString title = "Add behavior";
	_inputDialog->setTitle(title);

	QString lbl = "Behavior name:";
	_inputDialog->setLabelName(lbl);

	_inputDialog->exec();

	if (_inputDialog->getResult() == TextInputDialog::CREATED){
		std::string behavior = _inputDialog->getInput();

		if (behavior != ""){
			QString qKey = _phraseGroupBox->currentText();
			std::string key = qKey.toStdString();

			//Add to the behavior list
			_behaviorList->addItem(_inputDialog->getQInput());

			emit onPhraseBehaviorCreated(key, behavior);
		}else{
			QMessageBox::information(this, "Error", "Unable to add behavior as nothing was entered.");
		}
	}
}

void GenericPhraseTab::editBehaviorButtonClicked()
{
	QString title = "Edit behavior";
	_inputDialog->setTitle(title);

	QString lbl = "Behavior name:";
	_inputDialog->setLabelName(lbl);

	QString input = _behaviorList->currentItem()->text();
	_inputDialog->setInput(input);

	_inputDialog->exec();

	if (_inputDialog->getResult() == TextInputDialog::CREATED){
		std::string behavior = _inputDialog->getInput();

		if (behavior != ""){
			QString qKey = _phraseGroupBox->currentText();
			std::string key = qKey.toStdString();

			std::string oldBehavior = _behaviorList->currentItem()->text().toStdString();
			_behaviorList->currentItem()->setText(_inputDialog->getQInput());

			emit onPhraseBehaviorRemoved(key, oldBehavior);
			emit onPhraseBehaviorCreated(key, behavior);
		}else{
			QMessageBox::information(this, "Error", "Unable to edit behavior as nothing was entered.");
		}
	}
}

void GenericPhraseTab::removeBehaviorButtonClicked()
{
	QMessageBox::StandardButton btn = QMessageBox::question(this, "Delete behavior", "Are you sure you want to delete this behavior?"
			,QMessageBox::Yes | QMessageBox::No);

	if (btn == QMessageBox::Yes){
		_behaviorList->removeItemWidget(_behaviorList->currentItem());

		std::string key = _phraseGroupBox->currentText().toStdString();
		std::string behavior = _behaviorList->currentItem()->text().toStdString();

		QListWidgetItem* item = _behaviorList->takeItem(_behaviorList->currentIndex().row());

		emit onPhraseBehaviorRemoved(key, behavior);
	}
}
