/*
 * Controller.h
 *
 *  Created on: 7 Aug 2013
 *      Author: alex
 */

#ifndef CONTROLLER_H_
#define CONTROLLER_H_

#include <Model.h>

#include <QObject>

#include <boost/shared_ptr.hpp>

class Controller : public QObject
{

	Q_OBJECT

public:
	Controller(boost::shared_ptr<Model> model) : QObject(),
												_modelPtr(model)
	{

	}

public slots:
	void onRequestGeneralPhraseGroup(const std::string& key);
	void onRequestGuessGamePhraseGroup(const std::string& key);
	void onRequestMimicGamePhraseGroup(const std::string& key);

private:
	boost::shared_ptr<Model> _modelPtr;

};

#endif /* CONTROLLER_H_ */
