/*
 * Window.h
 *
 *  Created on: 5 Aug 2013
 *      Author: rapid
 */

#ifndef WINDOW_H_
#define WINDOW_H_

#include <BaseSettingsTab.h>
#include <BehaviorTab.h>
#include <AllPhrasesTab.h>
#include <Model.h>
#include <Controller.h>

#include <QWidget>
#include <QTabWidget>

#include <boost/shared_ptr.hpp>

#define WINDOW_TITLE "Settings editor"

class Window : public QWidget
{

public:
	Window(boost::shared_ptr<Controller> controller, boost::shared_ptr<Model> model) : QWidget()
	{
		init(controller, model);
	}

private:
	QTabWidget* _tabs;

	BaseSettingsTab* _baseSettingsTab;
	BehaviorTab* _behaviorTab;
	AllPhrasesTab* _phrasesTab;

//	PhraseTab* _phraseTab;
//	GuessGameTab* _guessGameTab;
//	MimicGameTab* _mimicGameTab;

	void init(boost::shared_ptr<Controller> controller, boost::shared_ptr<Model> model);

};


#endif /* WINDOW_H_ */
