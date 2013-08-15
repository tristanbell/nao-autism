/*
 * GuessGame.h
 *
 *  Created on: 30 Jul 2013
 *      Author: alex
 */

#ifndef GUESSGAME_H_
#define GUESSGAME_H_

class GuessGame : public Game {

public:
	GuessGame(GameSettings& settings);

	void startGame(void);
	void perform(void);
	void endGame(void);

private:
	// Is this needed?
	bool _waitingSpeech;
};

#endif /* GUESSGAME_H_ */
