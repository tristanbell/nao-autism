/*
 * main.cpp
 *
 *  Created on: 29 Jul 2013
 *      Author: tristan
 */

#include <Phrase.h>

#include <iostream>

int main()
{
	Phrase phrase("hey my name is %");

	std::list<std::string> l;
	l.push_back("alex");

	std::cout << phrase.getPhrase(l) << "\n";

	return 0;
}
