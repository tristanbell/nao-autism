/*
 * Window.h
 *
 *  Created on: 21 Aug 2013
 *      Author: alex
 */

#ifndef WINDOW_H_
#define WINDOW_H_

#include <QWidget>

class Window : public QWidget
{

public:
	Window() : QWidget()
	{
		init();
	}

private:
	void init();

};

#endif /* WINDOW_H_ */
