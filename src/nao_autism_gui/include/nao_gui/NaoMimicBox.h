/*
 * NaoMimicBox.h
 *
 *  Created on: 1 Jul 2013
 *      Author: alex
 */

#ifndef NAOMIMICBOX_H_
#define NAOMIMICBOX_H_

#include <nao_control/NaoControl.h>

#include <QGroupBox>
#include <QString>

#include <string>

namespace nao_gui{

const QString MIMIC_BOX_TITLE = "Mimic";

const std::string MIMIC_PROMPT = "Copy me";

const std::string MIMIC_CORRECT_ANSWER = "Well done";
const std::string MIMIC_INCORRECT_ANSWER = "Better luck next time";

class NaoMimicBox : public QGroupBox
{
	Q_OBJECT

public:
	NaoMimicBox(nao_control::NaoControl* control){
		setTitle(MIMIC_BOX_TITLE);

		naoControl = control;

		init();
	}

private:
	nao_control::NaoControl* naoControl;

	void init();

private Q_SLOTS:
	void promptButtonPressed();
	void correctButtonPressed();
	void incorrectButtonPressed();

Q_SIGNALS:
	void speechPerformed();

};

}

#endif /* NAOMIMICBOX_H_ */
