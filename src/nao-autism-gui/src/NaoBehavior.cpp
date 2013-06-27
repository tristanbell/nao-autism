#include <NaoBehavior.h>

NaoBehavior::NaoBehavior(QString name, QString behaviorName, std::vector<NaoSpeech> allSpeech)
{
	this->qName = name;
	this->qBehaviorName = behaviorName;

	this->behaviorName = behaviorName.toStdString();

	this->allSpeech = allSpeech;
}

QString NaoBehavior::getQName()
{
	return qName;
}

QString NaoBehavior::getQBehaviorName()
{
	return qBehaviorName;
}

std::string NaoBehavior::getBehaviorName(){
	return behaviorName;
}

std::vector<NaoSpeech> NaoBehavior::getSpeeches()
{
	return allSpeech;
}
