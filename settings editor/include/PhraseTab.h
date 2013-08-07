/*
 * PhraseTab.h
 *
 *  Created on: 5 Aug 2013
 *      Author: rapid
 */

#ifndef PHRASETAB_H_
#define PHRASETAB_H_

#include <AddBehaviorDialog.h>
#include <PhrasesWidget.h>

#include <QTabWidget>
#include <QString>

class PhraseTab : public QTabWidget
{

	Q_OBJECT

public:
	static const QString TAB_NAME;

	PhraseTab() : QTabWidget()
	{
		init();
	}

public slots:
	void onPhraseGroupLoaded(std::map<QString, PhraseGroupData>&);

private:
	PhrasesWidget* _phrasesWidget;

	void init();

};

#endif /* PHRASETAB_H_ */
