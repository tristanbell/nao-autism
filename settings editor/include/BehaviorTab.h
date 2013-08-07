/*
 * BehaviorTab.h
 *
 *  Created on: 5 Aug 2013
 *      Author: rapid
 */

#ifndef BEHAVIORTAB_H_
#define BEHAVIORTAB_H_

#include <AddBehaviorDialog.h>

#include <QTabWidget>
#include <QString>

#include <QDialog>
#include <QLabel>
#include <QComboBox>
#include <QListWidget>
#include <QLineEdit>

class BehaviorTab : public QTabWidget
{

	Q_OBJECT

public:
	static const QString TAB_NAME;

	BehaviorTab() : QTabWidget()
	{
		init();
	}

public slots:
	void onCreateBehaviorBtnClicked();

private:
	QComboBox* _behaviorNamesBox;

	QListWidget* _availableBehaviorList;
	QPushButton* _addBtn;
	QPushButton* _removeBtn;

	AddBehaviorDialog* _addBehaviorDialog;

	void init();

};

#endif /* BEHAVIORTAB_H_ */
