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
	void GuessGame::perform(void);

protected:
	void GuessGame::startGame(void);
	void GuessGame::endGame(void);

};

#endif /* GUESSGAME_H_ */
