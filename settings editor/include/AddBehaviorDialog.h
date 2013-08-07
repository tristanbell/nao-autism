/*
 * AddBehaviorDialog.h
 *
 *  Created on: 5 Aug 2013
 *      Author: rapid
 */

#ifndef ADDBEHAVIORDIALOG_H_
#define ADDBEHAVIORDIALOG_H_

#include <QDialog>

#include <QLineEdit>
#include <QPushButton>

class AddBehaviorDialog : public QDialog
{

public:
	static const QString DIALOG_TITLE_NAME;

	AddBehaviorDialog() : QDialog()
	{
		init();
	}

private:
	QLineEdit* _behaviorName;
	QPushButton* _createBtn;

	void init();

};


#endif /* ADDBEHAVIORDIALOG_H_ */
