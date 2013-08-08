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

class Controller : public QObject
{

	Q_OBJECT

public:
	Controller(Model* model) : QObject()
	{
		this->_model = model;
	}

public slots:
	void onGeneralPhraseCreated(std::string& key, std::string& phrase);
	void onGuessGamePhraseCreated(std::string& key, std::string& phrase);
	void onMimicGamePhraseCreated(std::string& key, std::string& phrase);

	void onGeneralPhraseBehaviorCreated(std::string& key, std::string& behavior);
	void onGuessGamePhraseBehaviorCreated(std::string& key, std::string& behavior);
	void onMimicGamePhraseBehaviorCreated(std::string& key, std::string& behavior);

	void onRequestGeneralPhraseGroup(const std::string& key);
	void onRequestGuessGamePhraseGroup(const std::string& key);
	void onRequestMimicGamePhraseGroup(const std::string& key);

private:
	Model* _model;

};

#endif /* CONTROLLER_H_ */
