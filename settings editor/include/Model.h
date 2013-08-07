/*
 * Model.h
 *
 *  Created on: 5 Aug 2013
 *      Author: rapid
 */

#ifndef MODEL_H_
#define MODEL_H_

#include <json/json.h>
#include <PhraseGroup.h>

#include <QObject>
#include <QString>
#include <map>

class Model : public QObject
{

	Q_OBJECT

public:
	Model(){ }

	void setData(Json::Value& doc);

private:
	Json::Value docRoot;

	void update();

signals:
	void behaviorsCleared();
	void generalPhrasesCleared();
	void guessGamePhrasesCleared();
	void mimicGamePhrasesCleared();

	void generalPhraseGroupLoaded(std::map<QString, PhraseGroupData>&);

};

#endif /* MODEL_H_ */
