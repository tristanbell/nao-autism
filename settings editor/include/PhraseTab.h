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

#include <string>

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
	void onPhraseGroupLoaded(const std::map<std::string, PhraseGroupData>&);
	void onPhraseGroupBoxIndexChanged(const QString& text);

	void onPhraseGroupRetrieved(const PhraseGroupData&);

private:
	PhrasesWidget* _phrasesWidget;

	void init();

signals:
	void onPhraseGroupRequired(const std::string& key);

};

#endif /* PHRASETAB_H_ */
