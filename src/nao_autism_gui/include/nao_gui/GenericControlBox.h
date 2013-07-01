/*
 * GenericControlBox.h
 *
 *  Created on: 1 Jul 2013
 *      Author: alex
 */

#ifndef GENERICCONTROLBOX_H_
#define GENERICCONTROLBOX_H_

#include <nao_control/NaoControl.h>

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

	void init();

private slots:
	void performPreviousSpeechClicked();
	void performPreviousBehaviorClicked();

};

}

#endif /* GENERICCONTROLBOX_H_ */
