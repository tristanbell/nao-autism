/*
 * GenericPhraseTab.h
 *
 *  Created on: 5 Aug 2013
 *      Author: rapid
 */

#ifndef GENERICPHRASETAB_H_
#define GENERICPHRASETAB_H_

#include <AddBehaviorDialog.h>
#include <TextInputDialog.h>
#include <PhraseGroupData.h>

#include <QTabWidget>
#include <QString>
#include <QComboBox>
#include <QListWidget>
#include <QPushButton>
#include <list>
#include <string>

#include <QWidget>

class GenericPhraseTab : public QTabWidget
{

	Q_OBJECT

public:
	static const QString TAB_NAME;

	GenericPhraseTab(QString tabName) : QTabWidget(),
								 	 	 _tabName(tabName)
	{
		init();
	}

	QString getTabName() const;

public slots:
	void phraseGroupBoxIndexChanged(const QString& text);

private:
	QString _tabName;

//	TextInputDialog* _addPhraseDialog;
//	TextInputDialog* _addBehaviorDialog;

	TextInputDialog* _inputDialog;

	QComboBox* _phraseGroupBox;

	QListWidget* _phrasesList;
	QPushButton* _addPhraseBtn;
	QPushButton* _editPhraseBtn;
	QPushButton* _removePhraseBtn;

	QListWidget* _behaviorList;
	QPushButton* _addBehaviorBtn;
	QPushButton* _editBehaviorBtn;
	QPushButton* _removeBehaviorBtn;

	void init();

	void loadIntoLists(const PhraseGroupData&);

	void setPhraseGroup(const std::map<std::string, PhraseGroupData>&);
	void setCurrentPhraseGroup(const PhraseGroupData&);

private slots:
	void onPhraseListItemChanged();

	void addPhraseButtonClicked();
	void editPhraseButtonClicked();
	void removePhraseButtonClicked();

	void onBehaviorListItemChanged();

	void addBehaviorButtonClicked();
	void editBehaviorButtonClicked();
	void removeBehaviorButtonClicked();

	void onPhraseGroupLoaded(const std::map<std::string, PhraseGroupData>&);
	void onPhraseGroupRetrieved(const PhraseGroupData&);

signals:
	void onPhraseGroupRequired(const std::string& key);

	void onPhraseCreated(std::string& key, std::string& phrase);
	void onPhraseRemoved(const std::string& key, const std::string& phrase);

	void onPhraseBehaviorCreated(std::string& key, std::string& behavior);
	void onPhraseBehaviorRemoved(const std::string& key, const std::string& behavior);

};

#endif /* GENERICPHRASETAB_H_ */
