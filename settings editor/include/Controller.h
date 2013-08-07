/*
 * Controller.h
 *
 *  Created on: 7 Aug 2013
 *      Author: alex
 */

#ifndef CONTROLLER_H_
#define CONTROLLER_H_

#include <Model.h>

class Controller
{

public:
	Controller(Model* model)
	{
		this->model = model;
	}

private:
	Model* model;

};

#endif /* CONTROLLER_H_ */
