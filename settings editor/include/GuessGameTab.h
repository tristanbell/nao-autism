/*
 * GuessGameTab.h
 *
 *  Created on: 5 Aug 2013
 *      Author: rapid
 */

#ifndef GUESSGAMETAB_H_
#define GUESSGAMETAB_H_

#include <PhraseGroupData.h>

#include <QTabWidget>
#include <QString>

#include <map>
#include <string>

class GuessGameTab
{

public:
	static const QString TAB_NAME;

	GuessGameTab()
	{

	}

	QString getTabName() const;

};

#endif /* GUESSGAMETAB_H_ */
