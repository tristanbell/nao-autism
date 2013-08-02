/*
 * Behavior.h
 *
 *  Created on: 31 Jul 2013
 *      Author: rapid
 */

#ifndef BEHAVIOR_H_
#define BEHAVIOR_H_

#include <string>

class Behavior
{

public:
	Behavior(std::string name);
	Behavior(std::string name, std::string actual);
	Behavior(std::string name, std::string actual, int classification);

	const std::string& getName() const;
	const std::string& getActualName() const;
	int getClassification() const;

private:
	std::string name;
	std::string actual;
	int classification;

};


#endif /* BEHAVIOR_H_ */
