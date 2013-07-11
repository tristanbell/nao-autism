/*
 * DataStore.h
 *
 * Implementing classes will determine how Training Data will be accessed
 * by the learner. Possible implementations are: Plain, Tree (binary search,
 * quad, etc), sorted
 *
 *  Created on: 10 Jul 2013
 *      Author: tristan
 */

#ifndef DATASTORE_H_
#define DATASTORE_H_

#include <DataPoint.h>
#include <vector>

namespace classification{

typedef std::vector<DataPoint*> TrainingData;

class DataStore {
public:
	virtual ~DataStore() { }

	/*
	 * Returns the DataPoint closest to otherPoint.
	 */
	virtual DataPoint *getDataPoint(const DataPoint *otherPoint) const = 0;
	/*
	 * Returns the k DataPoints which are closest to otherPoint.
	 */
	virtual std::vector<DataPoint*> getDataPoints(const DataPoint *otherPoint, const int k) const = 0;

protected:
	TrainingData trainingData;
};

}


#endif /* DATASTORE_H_ */
