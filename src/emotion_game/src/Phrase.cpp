#include <Phrase.h>

#include <iostream>
#include <sstream>

#include <cstdlib>

Phrase::Phrase(std::string phrase) : _phraseParts(), _behaviorNames()
{
	init(phrase);
}

Phrase::Phrase(std::string phrase, std::vector<std::string>& behaviorVector) : _phraseParts()
{
	_behaviorNames = behaviorVector;

	init(phrase);
}

void Phrase::init(std::string& phrase)
{
	bool partFound = false;
	int first = 0;

	//Go through the phrase and look for the parts
	for (int i=0;i<phrase.size();i++){
		char c = phrase.at(i);

		//We have found a part
		if (c == PHRASE_PART_DELIM){
			partFound = true;

			_phraseParts.push_back(phrase.substr(first, i-first));
			_phraseParts.push_back(PHRASE_PART_DELIM_STR);

			first = i + 1;
		}
	}

	if (first != phrase.size()){
		//Push the remainder of the string onto list
		_phraseParts.push_back(phrase.substr(first));
	}else if (!partFound){
		//No parts found, push whole string onto list
		_phraseParts.push_back(phrase);
	}
}

std::string Phrase::getPhrase() const
{
	if (_phraseParts.size() > 1){
		//Parts are required
		throw new MissingPhraseException(amountOfParts(), 0);
	}

	return _phraseParts.front();
}

std::string Phrase::getPhrase(std::list<std::string> parts) const
{
	//Check whether we have enough parts
	if (amountOfParts() <= parts.size()){
		std::stringstream ss;

		//Retrieve relevant iterators
		std::list<std::string>::iterator it = parts.begin();
		std::list<std::string>::const_iterator _it = _phraseParts.begin();

		//Iterate through each phrase, if it is the delimiter then replace it with
		//the next string in the parts list
		while (_it != _phraseParts.end()){
			std::string str = *_it;

			if (str == PHRASE_PART_DELIM_STR){
				ss << *it;
				it++;
			}else{
				ss << str;
			}

			_it++;
		}

		//Return the 'built' string
		return ss.str();
	}else{
		throw new MissingPhraseException(amountOfParts(), parts.size());
	}
}

const std::vector<std::string>& Phrase::getBehaviorNames() const
{
	return _behaviorNames;
}

std::string Phrase::getRandomBehaviorName() const
{
	int amount = getNumberOfBehaviors();

	if (amount == 0)
		return "";

	return _behaviorNames[rand() % amount];
}

int Phrase::getNumberOfBehaviors() const
{
	return _behaviorNames.size();
}

int Phrase::amountOfParts() const
{
	return (_phraseParts.size() != 0) ? _phraseParts.size() / 2 : 0;
}

bool Phrase::requiresParts() const
{
	return amountOfParts() > 0;
}
