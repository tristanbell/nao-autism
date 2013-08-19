/*
 * Window.h
 *
 *  Created on: 5 Aug 2013
 *      Author: rapid
 */

#ifndef WINDOW_H_
#define WINDOW_H_

#include <FileMenu.h>
#include <BaseSettingsTab.h>
#include <BehaviorsTab.h>
#include <PhrasesTab.h>
#include <Model.h>
#include <Controller.h>

#include <QMainWindow>
#include <QTabWidget>

#include <boost/shared_ptr.hpp>

#define WINDOW_TITLE "Settings editor"

class Window : public QMainWindow
{

public:
	Window(boost::shared_ptr<Controller> controller, boost::shared_ptr<Model> model) : QMainWindow()
	{
		init(controller, model);
	}

private:
	FileMenu* _fileMenu;

	QTabWidget* _tabs;

	BaseSettingsTab* _baseSettingsTab;
	BehaviorsTab* _behaviorsTab;
	PhrasesTab* _phrasesTab;

//	PhraseTab* _phraseTab;
//	GuessGameTab* _guessGameTab;
//	MimicGameTab* _mimicGameTab;

	void init(boost::shared_ptr<Controller> controller, boost::shared_ptr<Model> model);

};


#endif /* WINDOW_H_ */
