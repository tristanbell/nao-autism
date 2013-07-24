#include <classification/DWKNearestNeighbour.h>

#include <list>
#include <utility>

int classification::DWKNearestNeighbour::classify(const DataPoint* p) const
{
	std::vector<DataPoint*> vect = getDatastore()->getDataPoints();

	// Pairs consisting of distances to points (distance, point)
	std::list<std::pair<float, const DataPoint*> > distToPoint;

	float total = 0;
	// Populate distToPoint
	for (int i=0;i<vect.size();i++){
		DataPoint* curr = vect[i];

		float dist = p->getDistance(*curr);
		std::pair<float, const DataPoint*> pair(dist, curr);
		total += dist;

		distToPoint.push_back(pair);
	}

	std::map<short, float> classificationList;

	std::list<std::pair<float, const DataPoint*> >::iterator it = distToPoint.begin();
	while (it != distToPoint.end()){
		std::pair<float, const DataPoint*> pair = *it;

		// If there aren't any elements with the current DataPoint's classification
		// in classificationList, insert the classification of that element.
		if (classificationList.find(pair.second->getClassification()) == classificationList.end()){
			classificationList.insert(std::pair<short, float>(pair.second->getClassification(), 0));
		}

		float& weight = classificationList.at(pair.second->getClassification());
		weight += (1 / (pair.first / total));

		it++;
	}

	short classification = 0;
	float highestVal = -10000000000;

	//Now, finally, find the classification with the highest weight
	std::map<short, float>::iterator mapIt = classificationList.begin();
	while (mapIt != classificationList.end()){
		std::pair<short, float> pair = *mapIt;

		if (pair.second > highestVal){
			classification = pair.first;
			highestVal = pair.second;
		}

		mapIt++;
	}

	return classification;
}
