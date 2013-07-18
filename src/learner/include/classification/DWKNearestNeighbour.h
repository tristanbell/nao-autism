/*
 * DWKNearestNeighbour.h
 *
 *  Created on: 18 Jul 2013
 *      Author: alex
 */

#ifndef DWKNEARESTNEIGHBOUR_H_
#define DWKNEARESTNEIGHBOUR_H_

#include <classification/DataStore.h>
#include <classification/Learner.h>

namespace classification{

class DWKNearestNeighbour : public Learner
{

public:
	DWKNearestNeighbour(DataStore* data) : Learner(data)
	{  }

	virtual int classify(const DataPoint* p) const;

};

}

#endif /* DWKNEARESTNEIGHBOUR_H_ */
