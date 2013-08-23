/*
 * RewardBehaviorsTab.h
 *
 *  Created on: 22 Aug 2013
 *      Author: alex
 */

#ifndef REWARDBEHAVIORSTAB_H_
#define REWARDBEHAVIORSTAB_H_

#include <TextInputDialog.h>

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
	TextInputDialog* _behaviorDialog;

	QListWidget* _behaviorList;

	QPushButton* _addBehaviorBtn;
	QPushButton* _editBehaviorBtn;
	QPushButton* _removeBehaviorBtn;

	void init();

private slots:
	void onBehaviorListLoaded(const std::list<std::string>&);
	void onBehaviorListItemChanged();

	void onAddBehaviorBtnClicked();
	void onEditBehaviorBtnClicked();
	void onRemoveBehaviorBtnClicked();

signals:
	void addBehavior(const std::string&);
	void removeBehavior(const std::string&);

};


#endif /* REWARDBEHAVIORSTAB_H_ */
