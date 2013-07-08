/*
 * NaoSpeechData.h
 *
 *  Created on: 8 Jul 2013
 *      Author: alex
 */

#ifndef NAOSPEECHDATA_H_
#define NAOSPEECHDATA_H_

#include <string>
#include <map>

class NaoSpeechData
{

public:
	NaoSpeechData();

	const std::string get(const std::string& key) const;

	static NaoSpeechData load(const char* fileName);

private:
	std::map<std::string, std::string> speechMap;

	bool insert(const std::string& key, const std::string& val);

};


#endif /* NAOSPEECHDATA_H_ */
