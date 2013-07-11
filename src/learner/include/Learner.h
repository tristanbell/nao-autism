/*
 * Learner.h
 *
 *  Created on: 11 Jul 2013
 *      Author: rapid
 */

#ifndef LEARNER_H_
#define LEARNER_H_

#include <DataPoint.h>
#include <DataStore.h>

namespace classification{

class Learner
{

public:
	Learner(const DataStore* dataStore)
	{
		this->dataStore = dataStore;
	}

	virtual int classify(const DataPoint* p) const = 0;

	const DataStore* getDatastore() const
	{
		return dataStore;
	}

	virtual ~Learner()
	{
		delete dataStore;
	}

private:
	DataStore* dataStore;

};

}


#endif /* LEARNER_H_ */
