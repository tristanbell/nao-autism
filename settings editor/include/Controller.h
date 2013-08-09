/*
 * Controller.h
 *
 *  Created on: 7 Aug 2013
 *      Author: alex
 */

#ifndef CONTROLLER_H_
#define CONTROLLER_H_

#include <Model.h>
#include <BaseSettingsData.h>

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
	void onBaseSettingsUpdated(const BaseSettingsData& data);

	void onOpenRequested(const std::string& location);
	void onSaveRequested();
	void onSaveAsRequested(const std::string& location);

	void onGeneralPhraseCreated(std::string& key, std::string& phrase);
	void onGuessGamePhraseCreated(std::string& key, std::string& phrase);
	void onMimicGamePhraseCreated(std::string& key, std::string& phrase);

	void onGeneralPhraseRemoved(const std::string& key, const std::string& phrase);
	void onGuessGamePhraseRemoved(const std::string& key, const std::string& phrase);
	void onMimicGamePhraseRemoved(const std::string& key, const std::string& phrase);

	void onGeneralPhraseBehaviorCreated(std::string& key, std::string& behavior);
	void onGuessGamePhraseBehaviorCreated(std::string& key, std::string& behavior);
	void onMimicGamePhraseBehaviorCreated(std::string& key, std::string& behavior);

	void onGeneralPhraseBehaviorRemoved(const std::string& key, const std::string& behavior);
	void onGuessGamePhraseBehaviorRemoved(const std::string& key, const std::string& behavior);
	void onMimicGamePhraseBehaviorRemoved(const std::string& key, const std::string& behavior);

	void onGameBehaviorCreated(const std::string& actualName, const std::string& behaviorName);
	void onGameBehaviorRemoved(const std::string& actualName, const std::string& behaviorName);

	void onRequestGeneralPhraseGroup(const std::string& key);
	void onRequestGuessGamePhraseGroup(const std::string& key);
	void onRequestMimicGamePhraseGroup(const std::string& key);

	void onRequestGameBehaviors(const std::string& name);

private:
	Model* _model;

};

#endif /* CONTROLLER_H_ */
