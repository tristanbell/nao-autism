/*
 * NaoSpeech.h
 *
 *  Created on: 26 Jun 2013
 *      Author: alex
 */

#ifndef NAOSPEECH_H_
#define NAOSPEECH_H_

#include <QString>
#include <string>

class NaoSpeech
{

public:
	NaoSpeech(QString name, QString speech);

	QString getQName() const;
	QString getQSpeech() const;

	std::string getSpeech() const;

private:
	QString qName;
	QString qSpeech;

	std::string speech;

};

#endif /* NAOSPEECH_H_ */
