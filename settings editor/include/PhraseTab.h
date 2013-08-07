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
#include <Controller.h>

#include <QTabWidget>
#include <QString>

#include <string>
#include <boost/shared_ptr.hpp>

class PhraseTab : public QTabWidget
{

	Q_OBJECT

public:
	static const QString TAB_NAME;

	PhraseTab(boost::shared_ptr<Controller> controller) : QTabWidget(),
														  _controllerPtr(controller)
	{
		init();
	}

public slots:
	void onPhraseGroupLoaded(std::map<std::string, PhraseGroupData>&);
	void onPhraseGroupBoxIndexChanged(const QString& text);

private:
	boost::shared_ptr<Controller> _controllerPtr;

	PhrasesWidget* _phrasesWidget;

	void init();

};

#endif /* PHRASETAB_H_ */
