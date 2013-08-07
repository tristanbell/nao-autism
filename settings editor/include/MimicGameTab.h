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

class MimicGameTab : public QTabWidget
{

	Q_OBJECT

public:
	static const QString TAB_NAME;

	MimicGameTab() : QTabWidget()
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


#endif /* MIMICGAMETAB_H_ */
