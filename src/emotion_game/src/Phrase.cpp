#include <Phrase.h>

#include <iostream>

Phrase::Phrase(std::string phrase) : phraseParts()
{
	bool partFound = false;
	int first = 0;

	//Go through the phrase and look for the parts
	for (int i=0;i<phrase.size();i++){
		char c = phrase.at(i);

		if (c == PHRASE_PART_DELIM){
			partFound = true;

			std::string str = phrase.substr(first, i - 1);

			first = i + 1;

			std::cout << str << '\n';

			phraseParts.push_back(str);
		}
	}

	//No parts found
	if (partFound)
		phraseParts.push_back(phrase);
}
