/*
 * GenericControlBox.h
 *
 *  Created on: 1 Jul 2013
 *      Author: alex
 */

#ifndef GENERICCONTROLBOX_H_
#define GENERICCONTROLBOX_H_

#include <rosbag_recorder/rosbag_recorder.h>
#include <nao_gui/NaoAutismWindow.h>
#include <nao_control/NaoControl.h>

#include <QPushButton>
#include <QGridLayout>

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

	RosbagRecorder* currentRecorder;

	QPushButton* performPreviousBehaviorBtn;
	QPushButton* performPreviousSpeechBtn;

	QPushButton* startRecordBtn;
	QPushButton* endRecordBtn;

	void init();

private Q_SLOTS:
	void performPreviousSpeechClicked();
	void performPreviousBehaviorClicked();

	void onBehaviorPerformed();
	void onSpeechPerformed();

	void startRecordingClicked();
	void endRecordingClicked();

};

}

#endif /* GENERICCONTROLBOX_H_ */
