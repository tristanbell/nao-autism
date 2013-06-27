/*
 * NaoBehavior.h
 *
 *  Created on: 26 Jun 2013
 *      Author: alex
 */

#ifndef NAOBEHAVIOR_H_
#define NAOBEHAVIOR_H_

#include <NaoSpeech.h>

#include <string>
#include <QString>

#include <vector>

class NaoBehavior
{

public:
	NaoBehavior(QString name, QString behaviorName, std::vector<NaoSpeech> naoSpeech);

	QString getQName();

	QString getQBehaviorName();
	std::string getBehaviorName();

	std::vector<NaoSpeech> getSpeeches();

private:
	QString qName;

	std::string behaviorName;
	QString qBehaviorName;

	std::vector<NaoSpeech> allSpeech;

};
#endif /* NAOBEHAVIOR_H_ */
