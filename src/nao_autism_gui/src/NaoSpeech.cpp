#include <NaoSpeech.h>

#include <ros/ros.h>

NaoSpeech::NaoSpeech(QString name, QString speech) : behaviorName("")
{
	qName = name;
	qSpeech = speech;

	this->speech = speech.toStdString();
}

NaoSpeech::NaoSpeech(QString name, QString speech, std::string behaviorName)
{
	qName = name;
	qSpeech = speech;

	this->speech = speech.toStdString();

	this->behaviorName = behaviorName;
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

bool NaoSpeech::hasBehavior() const
{
	return behaviorName != "";
}

std::string NaoSpeech::getBehaviorName() const
{
	return behaviorName;
}
