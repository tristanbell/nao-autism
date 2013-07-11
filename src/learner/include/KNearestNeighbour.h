/*
 * KNearestNeighbour.h
 *
 *  Created on: 11 Jul 2013
 *      Author: rapid
 */

#ifndef KNEARESTNEIGHBOUR_H_
#define KNEARESTNEIGHBOUR_H_

#define DEFAULT_VALUE_K 5 //default value of k

#include <Learner.h>

namespace classification{

class KNearestNeighbour : public Learner
{

public:
	KNearestNeighbour(const DataStore* ds)
	{
			dataStore = ds;
			defaultK = DEFAULT_VALUE_K;
	}

	KNearestNeighbour(const DataStore* ds, int k)
	{
		dataStore = ds;
		defaultK = k;
	}

	virtual int classify(const DataPoint* p) const;
	virtual int classify(const DataPoint* p, const int& k) const;

private:
	int defaultK;

};

}

#endif /* KNEARESTNEIGHBOUR_H_ */