/*
 * FileTest.cpp
 *
 *  Created on: Aug 30, 2013
 *      Author: parallels
 */

#include <iostream>
#include <fstream>
#include <ros/ros.h>

const char* LOG_FILE_NAME = "~/nao-autism/test.log";
std::string BEHAVIOR = "happy";

class FileTest {
public:

	void writeToLogBehavior()
	{
		std::ofstream stream;

		stream.open(LOG_FILE_NAME, std::ios::app | std::ios::out);

		stream << "[" << ros::Time::now() << "] ";

		stream << "BEHAVIOR_BUTTON ";

		stream << "BEHAVIOR_NAME=" << BEHAVIOR << ' ';

		stream << "PROMPT_ENABLED=TRUE\n";

		stream.close();
	}

	void writeToLogPrompt()
	{
		printf("Writing prompt.\n");
		std::ofstream stream;

		stream.open(LOG_FILE_NAME, std::ios::app | std::ios::out);

		stream << "[" << ros::Time::now() << "] ";

		stream << "PROMPT_BUTTON ";

		stream << "BEHAVIOR_NAME=" << BEHAVIOR << '\n';

		stream.close();
		printf("Prompt written.\n");
	}

	void writeToLogAnswer(const bool& ans)
	{
		std::ofstream stream;

		stream.open(LOG_FILE_NAME, std::ios::app | std::ios::out);

		stream << "[" << ros::Time::now() << "] ";

		if (ans){
			stream << "CORRECT_BUTTON ";
		}else{
			stream << "INCORRECT_BUTTON ";
		}

		stream << "BEHAVIOR_NAME=" << BEHAVIOR << '\n';

		stream.close();
	}
};

int main(int argc, char **argv) {
	ros::init(argc, argv, "test");
	ros::Time::init();
	FileTest ft;
	ft.writeToLogBehavior();
	ft.writeToLogPrompt();
	ft.writeToLogAnswer(true);

	return 0;
}


