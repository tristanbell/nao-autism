/*
 * Window.h
 *
 *  Created on: 21 Aug 2013
 *      Author: alex
 */

#ifndef WINDOW_H_
#define WINDOW_H_

#include <QWidget>

#include <NodeBox.h>
#include <GeneralInformationBox.h>
#include <ExecutionControlBox.h>

class Window : public QWidget
{

public:
	Window() : QWidget()
	{
		init();
	}

private:
	NodeBox* _nodeBox;
	GeneralInformationBox* _generalInformationBox;
	ExecutionControlBox* _executionControlBox;

	void init();

};

#endif /* WINDOW_H_ */
