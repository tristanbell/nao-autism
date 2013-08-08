/*
 * AllPhrasesTab.h
 *
 *  Created on: 8 Aug 2013
 *      Author: rapid
 */

#ifndef ALLPHRASESTAB_H_
#define ALLPHRASESTAB_H_

#include <GenericPhraseTab.h>

#include <QTabWidget>
#include <QString>

class AllPhrasesTab : public QTabWidget
{

public:
	static const QString TAB_NAME;

	AllPhrasesTab()
	{
		init();
	}

	GenericPhraseTab* getGeneralPhraseTab();
	GenericPhraseTab* getGuessGamePhraseTab();
	GenericPhraseTab* getMimicGamePhraseTab();

private:
	GenericPhraseTab* _generalPhraseTab;
	GenericPhraseTab* _guessGamePhraseTab;
	GenericPhraseTab* _mimicGamePhraseTab;

	void init();

};


#endif /* ALLPHRASESTAB_H_ */
