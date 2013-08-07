/*
 * BaseSettingsTab.h
 *
 *  Created on: 5 Aug 2013
 *      Author: rapid
 */

#ifndef BASESETTINGSTAB_H_
#define BASESETTINGSTAB_H_

#include <QTabWidget>
#include <QString>

#include <QLabel>
#include <QLineEdit>

class BaseSettingsTab : public QTabWidget
{

public:
	static const QString TAB_NAME;

	BaseSettingsTab() : QTabWidget()
	{
		init();
	}

private:
	QLineEdit* _waitBox;
	QLineEdit* _timeoutBox;
	QLineEdit* _triesBox;

	void init();

};


#endif /* BASESETTINGSTAB_H_ */
