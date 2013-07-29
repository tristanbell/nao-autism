/*
 * Phrase.h
 *
 *  Created on: 29 Jul 2013
 *      Author: alex
 */

#ifndef PHRASE_H_
#define PHRASE_H_

#include <string>
#include <vector>

#define PHRASE_PART_DELIM '%'

class Phrase
{

public:
	Phrase(std::string phrase);

//	std::string getPhrase() const;
//	std::string getPhrase(std::vector<std::string> parts) const;
//
//	bool hasParts() const;

private:
	std::vector<std::string> phraseParts;

};


#endif /* PHRASE_H_ */
