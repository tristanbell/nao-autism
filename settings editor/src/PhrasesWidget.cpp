#include <PhrasesWidget.h>

#include <QGridLayout>
#include <QLabel>
#include <iostream>

#include <QString>

void PhrasesWidget::init()
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
	phraseBtnLayout->addWidget(_addPhraseBtn, 0, 0);

	_removePhraseBtn = new QPushButton("Remove phrase");
	phraseBtnLayout->addWidget(_removePhraseBtn, 0, 1);

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
	behaviorBtnLayout->addWidget(_addBehaviorBtn, 0, 0);

	_removeBehaviorBtn = new QPushButton("Remove behavior");
	behaviorBtnLayout->addWidget(_removeBehaviorBtn, 0, 1);

	//Add behavior add/remove button layout to the main behavior layout
	behaviorLayout->addLayout(behaviorBtnLayout, 2, 0);

	//Add main behavior layout to the main layout
	layout->addLayout(behaviorLayout, 3, 0);

	//Create instances of the dialogs
	QString phraseDialogTitle = QString::fromStdString("Add phrase");
	QString phraseDialogLabel = QString::fromStdString("Phrase:");

	QString behaviorDialogTitle = QString::fromStdString("Add behavior");
	QString behaviorDialogLabel = QString::fromStdString("Behavior:");

	_addPhraseDialog = new TextInputDialog(phraseDialogTitle, phraseDialogLabel);
	_addBehaviorDialog = new TextInputDialog(behaviorDialogTitle, behaviorDialogLabel);

	//Connect signals to required slots
	QObject::connect(_phraseGroupBox, SIGNAL(currentIndexChanged(const QString&)),
			this, SLOT(phraseGroupBoxIndexChanged(const QString&)));

	QObject::connect(_addPhraseBtn, SIGNAL(clicked()),
			this, SLOT(addPhraseButtonClicked()));
	QObject::connect(_removePhraseBtn, SIGNAL(clicked()),
			this, SLOT(removePhraseButtonClicked()));

	QObject::connect(_addBehaviorBtn, SIGNAL(clicked()),
			this, SLOT(addBehaviorButtonClicked()));
	QObject::connect(_removeBehaviorBtn, SIGNAL(clicked()),
			this, SLOT(removeBehaviorButtonClicked()));
}

void PhrasesWidget::setPhraseGroup(std::map<std::string, PhraseGroupData>& phraseGroups)
{
	//Clear old phrases
	_phraseGroupBox->clear();

	//Insert new phrases into box
	std::map<std::string, PhraseGroupData>::iterator it = phraseGroups.begin();
	while (it != phraseGroups.end()){
		std::pair<std::string, PhraseGroupData> pair = *it;

		QString qName = QString::fromStdString(pair.first);
		_phraseGroupBox->addItem(qName);

		it++;
	}
}

void PhrasesWidget::setCurrentPhraseGroup(const PhraseGroupData& data)
{
	loadIntoLists(data);
}

void PhrasesWidget::loadIntoLists(const PhraseGroupData& data)
{
	_phrasesList->clear();
	_behaviorList->clear();

	//Load phrases
	std::list<std::string>::const_iterator it = data.phraseVector.begin();
	while (it != data.phraseVector.end()){
		const std::string& current = *it;

		QString qString = QString::fromStdString(current);
		_phrasesList->addItem(qString);

		it++;
	}

	//Load behaviors
	it = data.behaviorVector.begin();
	while (it != data.behaviorVector.end()){
		const std::string& current = *it;

		QString qString = QString::fromStdString(current);
		_behaviorList->addItem(qString);

		it++;
	}
}

void PhrasesWidget::phraseGroupBoxIndexChanged(const QString& text)
{
	emit currentPhraseGroupIndexChanged(text);
}

void PhrasesWidget::addPhraseButtonClicked()
{
	_addPhraseDialog->exec();
}

void PhrasesWidget::removePhraseButtonClicked()
{

}

void PhrasesWidget::addBehaviorButtonClicked()
{
	_addBehaviorDialog->exec();
}

void PhrasesWidget::removeBehaviorButtonClicked()
{

}
