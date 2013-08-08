/*
 * GenericPhraseTab.h
 *
 *  Created on: 5 Aug 2013
 *      Author: rapid
 */

#ifndef GENERICPHRASETAB_H_
#define GENERICPHRASETAB_H_

#include <AddBehaviorDialog.h>
#include <PhrasesWidget.h>

#include <QTabWidget>
#include <QString>

#include <string>

class GenericPhraseTab : public QTabWidget
{

	Q_OBJECT

public:
	static const QString TAB_NAME;

	GenericPhraseTab(QString tabName) : QTabWidget(),
								 	 	 _tabName(tabName)
	{
		init();
	}

	QString getTabName() const;

public slots:
	void phraseCreated(std::string& key, std::string& phrase);
	void phraseBehaviorCreated(std::string& key, std::string& phrase);

	void onPhraseGroupLoaded(const std::map<std::string, PhraseGroupData>&);
	void onPhraseGroupBoxIndexChanged(const QString& text);

	void onPhraseGroupRetrieved(const PhraseGroupData&);

private:
	QString _tabName;
	PhrasesWidget* _phrasesWidget;

	void init();

signals:
	void onPhraseGroupRequired(const std::string& key);

	void onPhraseCreated(std::string& key, std::string& phrase);
	void onPhraseBehaviorCreated(std::string& key, std::string& phrase);

};

#endif /* GENERICPHRASETAB_H_ */
