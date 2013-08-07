/*
 * BehaviorData.h
 *
 *  Created on: 7 Aug 2013
 *      Author: rapid
 */

#ifndef BEHAVIORDATA_H_
#define BEHAVIORDATA_H_

#include <list>
#include <string>

struct BehaviorData{

	std::string _actualName;
	std::list<std::string> _behaviorNames;
	int _classification;

};

#endif /* BEHAVIORDATA_H_ */
