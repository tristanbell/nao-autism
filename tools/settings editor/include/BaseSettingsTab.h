/*
 * BaseSettingsTab.h
 *
 *  Created on: 5 Aug 2013
 *      Author: rapid
 */

#ifndef BASESETTINGSTAB_H_
#define BASESETTINGSTAB_H_

#include <BaseSettingsData.h>

#include <QPushButton>
#include <QTabWidget>
#include <QString>
#include <QLabel>
#include <QSpinBox>

class BaseSettingsTab : public QTabWidget
{

	Q_OBJECT

public:
	static const QString TAB_NAME;

	BaseSettingsTab() : QTabWidget()
	{
		init();
	}

private:
	QSpinBox* _waitBox;
	QSpinBox* _timeoutBox;
	QSpinBox* _triesBox;
	QSpinBox* _emotionsBeforeQuestionBox;
	QSpinBox* _confidenceBox;
	QPushButton* _setSettingsBtn;

	void init();

	void connectAll();
	void disconnectAll();

private slots:
	void onValueUpdated(int value);

	void onSettingsLoaded(const BaseSettingsData& data);

signals:
	void onSettingsUpdated(const BaseSettingsData& data);

};


#endif /* BASESETTINGSTAB_H_ */
