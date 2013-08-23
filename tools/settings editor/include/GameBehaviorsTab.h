/*
 * GameBehaviorsTab.h
 *
 *  Created on: 5 Aug 2013
 *      Author: rapid
 */

#ifndef GAMEBEHAVIORSTAB_H_
#define GAMEBEHAVIORSTAB_H_

#include <TextInputDialog.h>
#include <BehaviorData.h>

#include <QTabWidget>
#include <QString>

#include <QDialog>
#include <QLabel>
#include <QComboBox>
#include <QListWidget>
#include <QLineEdit>

class GameBehaviorsTab : public QTabWidget
{

	Q_OBJECT

public:
	static const QString TAB_NAME;

	GameBehaviorsTab() : QTabWidget()
	{
		init();
	}

public slots:
	void onBehaviorBoxIndexChanged(const QString&);
	void onBehaviorListItemChanged();

	void onCreateBehaviorBtnClicked();
	void onEditBehaviorBtnClicked();
	void onRemoveBehaviorBtnClicked();

	void onBehaviorListLoaded(const std::list<BehaviorData>&);
	void onBehaviorDataRetrieved(const BehaviorData&);

private:
	QComboBox* _behaviorNamesBox;

	QListWidget* _availableBehaviorList;
	QPushButton* _addBtn;
	QPushButton* _editBtn;
	QPushButton* _removeBtn;

	TextInputDialog* _behaviorDialog;

	void init();
	void fillList(const BehaviorData&);

signals:
	void onBehaviorCreated(const std::string& key, const std::string& behaviorName);
	void onBehaviorRemoved(const std::string& key, const std::string& behaviorName);

	void behaviorDataRequired(const std::string& name);

};

#endif /* GAMEBEHAVIORTAB_H_ */
