/*
 * MimicGame.h
 *
 *  Created on: 30 Jul 2013
 *      Author: alex
 */

#ifndef MIMICGAME_H_
#define MIMICGAME_H_

class MimicGame : public Game {

public:
	MimicGame(GameSettings&);

	void startGame(void);
	void perform(void);
	void endGame(void);

};

#endif /* MIMICGAME_H_ */
