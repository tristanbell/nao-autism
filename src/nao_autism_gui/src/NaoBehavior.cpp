#include <NaoBehavior.h>

NaoBehavior::NaoBehavior(QString name, QString behaviorName)
{
	this->qName = name;
	this->qBehaviorName = behaviorName;

	this->behaviorName = behaviorName.toStdString();
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
