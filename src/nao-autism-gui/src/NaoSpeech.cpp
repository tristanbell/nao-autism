#include <NaoSpeech.h>

#include <ros/ros.h>

NaoSpeech::NaoSpeech(QString name, QString speech)
{
	qName = name;
	qSpeech = speech;

	this->speech = speech.toStdString();
}

QString NaoSpeech::getQName()
{
	return qName;
}

QString NaoSpeech::getQSpeech()
{
	return qSpeech;
}

std::string NaoSpeech::getSpeech()
{
	return speech;
}
