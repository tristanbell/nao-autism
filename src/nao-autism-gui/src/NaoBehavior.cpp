#include <NaoBehavior.h>

NaoBehavior::NaoBehavior(QString name,QString behaviorName)
{
	this->qName = name;
	this->qBehaviorName = behaviorName;

	this->behaviorName = behaviorName.toStdString();
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
