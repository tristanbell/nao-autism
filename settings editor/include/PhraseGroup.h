/*
 * Phrase.h
 *
 *  Created on: 7 Aug 2013
 *      Author: alex
 */

#ifndef PHRASE_H_
#define PHRASE_H_

#include <QString>
#include <list>

struct PhraseGroupData
{
	std::list<QString> phraseVector;
	std::list<QString> behaviorVector;
};


#endif /* PHRASE_H_ */
