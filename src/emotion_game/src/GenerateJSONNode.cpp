/*
 * This ROS node is used to generate the required JSON files.
 *
 * The node must be ran as follows:
 * 		rosrun emotion_game gen_json <file>
 * Where <file> is the path to the file that the json data will be
 * wrote to.
 */
#include <GenerateJSONFile.h>

#include <ros/ros.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "json_gen");

	if (argc == 2){
		std::cout << "Generating JSON data." << std::endl;

		Json::Value doc = generateAllData();

		Json::StyledWriter writer;
		std::string result = writer.write(doc);

		std::cout << "Generation finished." << std::endl;

		//Write json data to file
		std::fstream fs;
		fs.open(argv[1], std::fstream::out);

		std::cout << "Writing json data to " << argv[1] << std::endl;

		//Write the json data and flush to ensure complete write
		fs << result;
		fs.flush();

		std::cout << "Successfully wrote generated json data.\n";

		//Close stream
		fs.close();
	}else{
		std::cout << "Invalid number of arguments, the following format must be used:" << std::endl;
		std::cout << "\trosrun emotion_game gen_json <file>" << std::endl;
		std::cout << "Where <file> is the file name to write the json data to." << std::endl;
	}

	ros::shutdown();

	return 0;
}
