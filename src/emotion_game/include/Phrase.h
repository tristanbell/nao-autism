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
#include <vector>

#define PHRASE_PART_DELIM '%'
#define PHRASE_PART_DELIM_STR "%"

class Phrase
{

public:
	Phrase(std::string phrase);
	Phrase(std::string phrase, std::vector<std::string>& behaviorVector);

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

	const std::vector<std::string>& getBehaviorNames() const;

	std::string getRandomBehaviorName() const;

	int getNumberOfBehaviors() const;

	/**
	 * Returns the amount of phrase parts required.
	 */
	int amountOfParts() const;

	/**
	 * Returns whether the given Phrase requires any parts. If a phrase requires
	 * parts then it must be given a list of strings in order to retrieve the phrase.
	 *
	 * Returns whether the given Phrase requires any parts.
	 */
	bool requiresParts() const;

private:
	std::list<std::string> _phraseParts;
	std::vector<std::string> _behaviorNames;

	void init(std::string& phrase);

};

class MissingPhraseException
{

public:
	MissingPhraseException(int size, int given)
	{
		_partsRequired = size;
		_partsGiven = given;
	}

	int _partsRequired;
	int _partsGiven;

};


#endif /* PHRASE_H_ */
