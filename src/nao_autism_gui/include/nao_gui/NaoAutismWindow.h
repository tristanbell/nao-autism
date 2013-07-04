/*
 * NaoAutismWindow.h
 *
 *  Created on: 26 Jun 2013
 *      Author: alex
 */

#ifndef NAOAUTISMWINDOW_H_
#define NAOAUTISMWINDOW_H_

#include <nao_control/NaoControl.h>
#include <NaoBehavior.h>
#include <NaoSpeech.h>

#include <QWidget>
#include <QString>

#include <vector>

namespace nao_gui{

const QString WINDOW_TITLE = "GUI";

/**
 * This class represents the window that the application runs it, it handles
 * initialising the core 'box' objects that define the sections of the GUI.
 */
class NaoAutismWindow : public QWidget
{

public:
	NaoAutismWindow(std::vector<NaoBehavior>&);

private:
	/**
	 * Called after the constructor has dealt with setting up the correct size/title
	 * of the window. This initialises the core widgets of the application and adds them to
	 * the layout.
	 */
	void init(std::vector<NaoBehavior>&);

};

}

#endif /* NAOAUTISMWINDOW_H_ */
