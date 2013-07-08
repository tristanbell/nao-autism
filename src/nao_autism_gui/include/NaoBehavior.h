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

class NaoBehavior
{

public:
	NaoBehavior(QString name, QString behaviorName);

	QString getQName() const;

	QString getQBehaviorName() const;
	std::string getBehaviorName() const;

private:
	QString qName;

	std::string behaviorName;
	QString qBehaviorName;

};
#endif /* NAOBEHAVIOR_H_ */
