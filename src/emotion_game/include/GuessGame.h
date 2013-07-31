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
	void perform(void);

protected:
	void startGame(void);
	void endGame(void);

};

#endif /* GUESSGAME_H_ */
