/*
 * RewardBehaviorsTab.h
 *
 *  Created on: 22 Aug 2013
 *      Author: alex
 */

#ifndef REWARDBEHAVIORSTAB_H_
#define REWARDBEHAVIORSTAB_H_

#include <QTabWidget>

#include <QListWidget>
#include <QPushButton>
#include <QLabel>

class RewardBehaviorsTab : public QTabWidget
{

	Q_OBJECT

public:
	static const QString TAB_NAME;

	RewardBehaviorsTab() : QTabWidget()
	{
		init();
	}

private:
	QListWidget* _behaviorList;

	QPushButton* _addBehaviorBtn;
	QPushButton* _editBehaviorBtn;
	QPushButton* _removeBehaviorBtn;

	void init();

};


#endif /* REWARDBEHAVIORSTAB_H_ */
