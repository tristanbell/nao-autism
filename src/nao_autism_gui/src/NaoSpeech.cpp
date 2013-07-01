#include <NaoSpeech.h>

#include <ros/ros.h>

NaoSpeech::NaoSpeech(QString name, QString speech)
{
	qName = name;
	qSpeech = speech;

	this->speech = speech.toStdString();
}

QString NaoSpeech::getQName() const
{
	return qName;
}

QString NaoSpeech::getQSpeech() const
{
	return qSpeech;
}

std::string NaoSpeech::getSpeech() const
{
	return speech;
}
