/*
 * NaoBehavior.h
 *
 *  Created on: 26 Jun 2013
 *      Author: alex
 */

#ifndef NAOBEHAVIOR_H_
#define NAOBEHAVIOR_H_

#include <string>
#include <QString>

#include <vector>

#include <NaoSpeech.h>

class NaoBehavior
{

public:
	NaoBehavior(QString name, QString behaviorName, std::vector<NaoSpeech> naoSpeech);

	QString getQName() const;

	QString getQBehaviorName() const;
	std::string getBehaviorName() const;

	const std::vector<NaoSpeech>& getSpeeches() const;

private:
	QString qName;

	std::string behaviorName;
	QString qBehaviorName;

	std::vector<NaoSpeech> allSpeech;

};
#endif /* NAOBEHAVIOR_H_ */
