/*
 * Controller.h
 *
 *  Created on: 7 Aug 2013
 *      Author: alex
 */

#ifndef CONTROLLER_H_
#define CONTROLLER_H_

#include <Model.h>

#include <boost/shared_ptr.hpp>

class Controller
{

public:
	Controller(boost::shared_ptr<Model> model) : _modelPtr(model)
	{

	}

	PhraseGroupData& getGeneralPhraseGroup(std::string key);
	const PhraseGroupData& getGeneralPhraseGroup(std::string key) const;

	PhraseGroupData& getGuessGamePhraseGroup(std::string key);
	const PhraseGroupData& getGuessGamePhraseGroup(std::string key) const;

	PhraseGroupData& getMimicGamePhraseGroup(std::string key);
	const PhraseGroupData& getMimicGamePhraseGroup(std::string key) const;

private:
	boost::shared_ptr<Model> _modelPtr;

};

#endif /* CONTROLLER_H_ */
