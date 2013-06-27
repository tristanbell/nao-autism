/*
 * NaoAutismWindow.h
 *
 *  Created on: 26 Jun 2013
 *      Author: alex
 */

#ifndef NAOAUTISMWINDOW_H_
#define NAOAUTISMWINDOW_H_

#include <NaoBehavior.h>
#include <NaoSpeech.h>

#include <QWidget>
#include <QString>

#include <QList>

const QString WINDOW_TITLE = "GUI";

class NaoAutismWindow : public QWidget
{

public:
	NaoAutismWindow(QList<NaoBehavior>*, QList<NaoSpeech>*);

private:
	void init(QList<NaoBehavior>*, QList<NaoSpeech>*);

};

#endif /* NAOAUTISMWINDOW_H_ */
