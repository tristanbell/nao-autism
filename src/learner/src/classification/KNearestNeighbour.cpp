#include <KNearestNeighbour.h>

#include <map>
#include <exception>

int classification::KNearestNeighbour::classify(const DataPoint* p) const
{
	return classify(p, defaultK);
}

int classification::KNearestNeighbour::classify(const DataPoint* p, const int& k) const
{
	std::vector<DataPoint*> points = dataStore->getDataPoints(p, k);

	//Construct map containing pairs of class to number of votes (where the class is the key)
	std::map<int, int> classificationVote;

	for (int i=0;i<points.size();i++){
		DataPoint* p = points[i];

		if (p != NULL){
			try{
				int& pClassNum = classificationVote.at(p->classification);
				pClassNum++;
			}catch(std::out_of_range* oof){
				classificationVote.insert(std::pair<int, int>(p->classification, 1));
			}
		}
	}

	//Find class that has the highest vote
	std::map<int, int>::iterator it = classificationVote.begin();
	std::pair<int, int> pair = *it;

	int highestClass = pair.first;
	int highestNum = pair.second;

	it++;

	for (int i=1;i<classificationVote.size();i++){
		pair = *it;

		if (pair.second > highestNum){
			highestClass = pair.first;
			highestNum = pair.second;
		}
	}

	return highestClass;
}
