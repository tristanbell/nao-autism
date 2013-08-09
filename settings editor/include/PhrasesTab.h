/*
 * PhrasesTab.h
 *
 *  Created on: 8 Aug 2013
 *      Author: rapid
 */

#ifndef PHRASESTAB_H_
#define PHRASESTAB_H_

#include <GenericPhraseTab.h>

#include <QTabWidget>
#include <QString>

class PhrasesTab : public QTabWidget
{

public:
	static const QString TAB_NAME;

	PhrasesTab()
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
