#include <Phrase.h>

#include <iostream>
#include <sstream>

Phrase::Phrase(std::string phrase) : phraseParts()
{
	bool partFound = false;
	int first = 0;

	//Go through the phrase and look for the parts
	for (int i=0;i<phrase.size();i++){
		char c = phrase.at(i);

		//We have found a part
		if (c == PHRASE_PART_DELIM){
			partFound = true;

			phraseParts.push_back(phrase.substr(first, i));
			phraseParts.push_back("%");

			first = i + 1;
		}
	}

	if (first != phrase.size()){
		//Push the remainder of the string onto list
		phraseParts.push_back(phrase.substr(first));
	}else if (!partFound){
		//No parts found, push whole string onto list
		phraseParts.push_back(phrase);
	}
}

std::string Phrase::getPhrase() const
{
	if (phraseParts.size() > 1){
		//Parts are required
		throw new MissingPhraseException(phraseParts.size() - 1, 0);
	}

	return phraseParts.front();
}

std::string Phrase::getPhrase(std::list<std::string> parts) const
{
	//Check whether we have enough parts
	if (phraseParts.size() - 1 <= parts.size()){
		std::stringstream ss;

		//Retrieve relevant iterators
		std::list<std::string>::iterator it = parts.begin();
		std::list<std::string>::const_iterator _it = phraseParts.begin();

		//Iterate through each phrase, if it is the delimiter then replace it with
		//the next string in the parts list
		while (_it != phraseParts.end()){
			std::string str = *_it;

			if (str == "%"){
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
		throw new MissingPhraseException(phraseParts.size() - 1, parts.size());
	}
}
