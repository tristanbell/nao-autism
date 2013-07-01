#include <NaoBehavior.h>

NaoBehavior::NaoBehavior(QString name, QString behaviorName, std::vector<NaoSpeech> allSpeech)
{
	this->qName = name;
	this->qBehaviorName = behaviorName;

	this->behaviorName = behaviorName.toStdString();

	this->allSpeech = allSpeech;
}

QString NaoBehavior::getQName() const
{
	return qName;
}

QString NaoBehavior::getQBehaviorName() const
{
	return qBehaviorName;
}

std::string NaoBehavior::getBehaviorName() const
{
	return behaviorName;
}

const std::vector<NaoSpeech>& NaoBehavior::getSpeeches() const
{
	return allSpeech;
}
