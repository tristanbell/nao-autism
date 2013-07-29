/*
 * Phrase.h
 *
 *  Created on: 29 Jul 2013
 *      Author: alex
 */

#ifndef PHRASE_H_
#define PHRASE_H_

#include <string>
#include <list>

#define PHRASE_PART_DELIM '%'

class Phrase
{

public:
	Phrase(std::string phrase);

	/**
	 * Returns the phrase currently being stored. If there are parts that must be filled then
	 * a MissingPhraseException will be thrown.
	 *
	 * Returns the phrase being stored.
	 */
	std::string getPhrase() const;

	/**
	 * Returns the phrase currently being stored, with any '%' parts replaced by the equivalent
	 * in the parts list.
	 *
	 * If the parts list is too small than 'phrases' are missing, thus, a MissingPhraseException is thrown,
	 * however, if the parts list is too big then it simply selects the first n strings to insert where n is
	 * equal to the number of '%' characters.
	 *
	 * Returns the phrase being stored.
	 */
	std::string getPhrase(std::list<std::string> parts) const;

	/**
	 * Returns whether the given Phrase requires any parts. If a phrase requires
	 * parts then it must be given a list of strings in order to retrieve the phrase.
	 *
	 * Returns whether the given Phrase requires any parts.
	 */
	bool requiresParts() const;

private:
	std::list<std::string> phraseParts;

};

class MissingPhraseException
{

public:
	MissingPhraseException(int size, int given)
	{
		partsRequired = size;
		partsGiven = given;
	}

	int partsRequired;
	int partsGiven;

};


#endif /* PHRASE_H_ */
