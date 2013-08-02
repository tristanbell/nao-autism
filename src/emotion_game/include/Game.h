/*
 * Game.h
 *
 *  Created on: 29 Jul 2013
 *      Author: tristan
 */

#ifndef GAME_H_
#define GAME_H_

#include <GameSettings.h>
#include <nao_control/NaoControl.h>

class Game {
public:
	Game(GameSettings gs) : _settings(gs),
							_naoControl()
	{ }

	virtual ~Game() { }

	GameSettings& getGameSettings()
	{
		return _settings;
	}

	nao_control::NaoControl& getNaoControl()
	{
		return _naoControl;
	}

	virtual void startGame(void) = 0;
	virtual void perform(void) = 0;
	virtual void endGame(void) = 0;

	bool isDone;

protected:
	nao_control::NaoControl _naoControl;
	GameSettings _settings;

	void say(const Phrase& phrase);
	void say(const Phrase& phrase, const std::list<std::string> parts);
	void sayRandParts(const Phrase& phrase, std::list<std::string> parts);
	void sayRandParts(const Phrase& phrase, const std::string& required, std::list<std::string> parts);

	void sayAny(const std::vector<Phrase>&);
	void sayAny(const std::vector<Phrase>& phraseVector, const std::list<std::string>& parts);
	void sayAnyRandParts(const std::vector<Phrase>& phraseVector, const std::list<std::string>& parts);
	void sayAnyRandParts(const std::vector<Phrase>& phraseVector, const std::string& required, const std::list<std::string>& parts);
};


#endif /* GAME_H_ */
