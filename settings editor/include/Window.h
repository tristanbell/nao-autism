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
#include <PhraseTab.h>
#include <GuessGameTab.h>
#include <MimicGameTab.h>
#include <Model.h>

#include <QWidget>
#include <QTabWidget>

#define WINDOW_TITLE "Settings editor"

class Window : public QWidget
{

public:
	Window(Model* model) : QWidget()
	{
		init(model);
	}

private:
	QTabWidget* _tabs;

	BaseSettingsTab* _baseSettingsTab;
	BehaviorTab* _behaviorTab;
	PhraseTab* _phraseTab;
	GuessGameTab* _guessGameTab;
	MimicGameTab* _mimicGameTab;

	void init(Model* model);

};


#endif /* WINDOW_H_ */
