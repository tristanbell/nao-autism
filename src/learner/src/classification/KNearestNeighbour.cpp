#include <classification/KNearestNeighbour.h>

#include <map>
#include <exception>

#include <iostream>

int classification::KNearestNeighbour::classify(const DataPoint* p) const
{
	return classify(p, defaultK);
}

int classification::KNearestNeighbour::classify(const DataPoint* p, const int& k) const
{
	std::vector<DataPoint*> points = getDatastore()->getDataPoints(p, k);

	//Construct map containing pairs of class to number of votes (where the class is the key)
	std::map<short, int> classificationVote;

	for (int i=0;i<points.size();i++){
		DataPoint* pt = points[i];

		if (pt != NULL){
			try{
				int& pClassNum = classificationVote.at(pt->getClassification());
				pClassNum++;
			}catch(std::out_of_range& oof){
				classificationVote.insert(std::pair<int, int>(pt->getClassification(), 1));
			}
		}
	}

	//Find class that has the highest vote by iterating through the map
	std::map<short, int>::iterator it = classificationVote.begin();
	std::pair<short, int> pair = *it;

	int highestClass = pair.first;
	int highestNum = pair.second;

	it++;

	while (it != classificationVote.end()){
		pair = *it;

		if (pair.second > highestNum){
			highestClass = pair.first;
			highestNum = pair.second;
		}

		it++;
	}

	return highestClass;
}
