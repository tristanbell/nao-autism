#include <NaoSpeechData.h>

#include <iostream>
#include <fstream>
#include <stdexcept>

NaoSpeechData::NaoSpeechData() : speechMap()
{

}

const std::string NaoSpeechData::get(const std::string& key) const
{
	try{
		return speechMap.at(key);
	}catch(std::out_of_range& rng){
		return "Speech not found";
	}
}

NaoSpeechData NaoSpeechData::load(const char* fileName)
{
	NaoSpeechData data;
	std::ifstream fs;

	fs.open(fileName, std::ios::in);

	while (fs.good() && !fs.eof() && !fs.fail()){
		std::string line;

		std::getline(fs, line);

		//We have reached the end of file
		if (line == "\0")
			break;

		int delimIndex = line.find(' ');

		std::string key = line.substr(0, delimIndex);
		std::string val = line.substr(delimIndex + 1, line.size());

		data.insert(key, val);

		std::cout << "Loaded: " << key << ", " << val << "\n";
	}

	fs.close();

	return data;
}

bool NaoSpeechData::insert(const std::string& key, const std::string& val)
{
	std::pair<std::map<std::string, std::string>::iterator, bool> rtn;

	rtn = speechMap.insert(std::pair<std::string, std::string>(key, val));

	return rtn.second;
}
