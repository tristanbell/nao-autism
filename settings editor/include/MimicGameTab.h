/*
 * MimicGameTab.h
 *
 *  Created on: 5 Aug 2013
 *      Author: rapid
 */

#ifndef MIMICGAMETAB_H_
#define MIMICGAMETAB_H_

#include <PhrasesWidget.h>
#include <PhraseGroupData.h>

#include <QTabWidget>
#include <QString>

#include <map>
#include <string>

class MimicGameTab
{

public:
	static const QString TAB_NAME;

	MimicGameTab()
	{

	}

	QString getTabName() const;

};


#endif /* MIMICGAMETAB_H_ */
