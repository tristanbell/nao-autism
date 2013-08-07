/*
 * PhraseTab.h
 *
 *  Created on: 5 Aug 2013
 *      Author: rapid
 */

#ifndef PHRASETAB_H_
#define PHRASETAB_H_

#include <QTabWidget>
#include <QString>

class PhraseTab : public QTabWidget
{

public:
	static const QString TAB_NAME;

	PhraseTab() : QTabWidget()
	{
		init();
	}

private:
	void init();

};

#endif /* PHRASETAB_H_ */
