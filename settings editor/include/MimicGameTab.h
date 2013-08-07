/*
 * MimicGameTab.h
 *
 *  Created on: 5 Aug 2013
 *      Author: rapid
 */

#ifndef MIMICGAMETAB_H_
#define MIMICGAMETAB_H_

#include <QTabWidget>
#include <QString>

class MimicGameTab : public QTabWidget
{

public:
	static const QString TAB_NAME;

	MimicGameTab() : QTabWidget()
	{
		init();
	}

private:
	void init();

};


#endif /* MIMICGAMETAB_H_ */
