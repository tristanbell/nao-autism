/*
 * GuessGame.h
 *
 *  Created on: 30 Jul 2013
 *      Author: alex
 */

#ifndef GUESSGAME_H_
#define GUESSGAME_H_

/**
 * Implementation of the 'Guessing' game.
 *
 * The guessing game is simple. The robot will perform an emotion, asking the child a question
 * regarding the previously performed emotion (such as: Was the robot happy or sad?) and the child
 * then needs to answer correctly to continue (if the child doesn't answer correctly, then the robot
 * will ask the question again, if the child doesn't answer at all then the robot will, again, ask
 * the question again). When either the child answered correctly or they have exceeded the maximum
 * amount of prompts, then the robot will ask the child if they want to continue.
 */
class GuessGame : public Game {

public:
	GuessGame(GameSettings& settings);

	void startGame(void);
	void perform(void);
	void endGame(void);

private:
	/**
	 * Asks question relating to the randomly performed behavior in the previous state.
	 * It will then set the state to WAITING_ANSWER_CONTINUE where it will either wait for
	 * the correct answer or wait until it has timed out (in which case it may ask the child
	 * the question again, or, ask the child if they want to continue playing this is performed
	 * by swapping to the ASK_QUESTION_CONTINUE state).
	 */
	void askQuestion();

	/**
	 * Handles the situation where the child gives the correct answer. It congratulates the
	 * child and then goes to the ASK_QUESTION_CONTINUE state.
	 */
	void handleCorrectAnswer();

	/**
	 * This function checks whether the string given in the parenthesis is an incorrect answer,
	 * if it is then a call will be made to handleIncorrectAnswer(). An incorrect answer is defined
	 * as a string that equals one of the behavior names but isn't the correct behavior for the given
	 * question.
	 */
	bool checkIncorrectAnswer(const std::string& answer);

	/**
	 * Handles the situation where the child gives an incorrect answer. It simply delegates the call
	 * to promptChild() whilst incrementing the 'timeout' counter.
	 */
	void handleIncorrectAnswer();

	/**
	 * Handles the situation where the child has exceeded the maximum amount of prompts.
	 * The robot will state that the child is incorrect and change the state to the ASK_QUESTION_CONTINUE
	 * state to see if the child wants to continue playing.
	 */
	void handleTimeout();

	/**
	 * Prompts the child (for example: says 'try again'), and changes the state to the ASK_QUESTION
	 * state to ask the given question again.
	 */
	void promptChild();

};

#endif /* GUESSGAME_H_ */
