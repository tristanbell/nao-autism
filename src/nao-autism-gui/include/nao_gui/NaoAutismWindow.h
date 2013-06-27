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

class NaoAutismWindow : public QWidget
{

public:
	NaoAutismWindow(std::vector<NaoBehavior>&);

private:
	nao_control::NaoControl naoControl;

	void init(std::vector<NaoBehavior>&);

};

}

#endif /* NAOAUTISMWINDOW_H_ */
