/*
 * GenericControlBox.h
 *
 *  Created on: 1 Jul 2013
 *      Author: alex
 */

#ifndef GENERICCONTROLBOX_H_
#define GENERICCONTROLBOX_H_

#include <nao_gui/NaoAutismWindow.h>
#include <nao_control/NaoControl.h>

#include <QPushButton>
#include <QGridLayout>
#include <QTimer>

namespace nao_gui{

class GenericControlBox : public QGridLayout
{
	Q_OBJECT

public:
	GenericControlBox(nao_control::NaoControl* control){
		naoControl = control;

		init();
	}

private:
	nao_control::NaoControl* naoControl;

	QPushButton* performPreviousBehaviorBtn;
	QPushButton* performPreviousSpeechBtn;

	QPushButton* startRecordBtn;

	QTimer* recordingTimer;

	void init();

private Q_SLOTS:
	void performPreviousSpeechClicked();
	void performPreviousBehaviorClicked();

	void onBehaviorPerformed();
	void onSpeechPerformed();

	void startRecordingClicked();
	void onRecordingStopped();

};

}

#endif /* GENERICCONTROLBOX_H_ */
