/*
 * dataLoader.h
 *
 *  Created on: 3 Jul 2013
 *      Author: tristan
 */

#ifndef DATALOADER_H_
#define DATALOADER_H_

#include <PoseData.h>
#include <classification/PoseDataPoint.h>
#include <classification/DataStore.h>
#include <vector>
#include <string>

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/filesystem.hpp>

#include <ros/ros.h>
#include <rosbag/bag.h>

namespace classification
{

class DataLoader
{
public:
	/*
	 * Gets a vector of DataPoints that are constructed from the bagfile at 'filename'.
	 */
	static std::vector<DataPoint*> loadData(std::string filename);

	/*
	 * Gets the subset of DataPoints in 'data' that correspond to the given start and end times.
	 */
	static std::vector<DataPoint*> getDataSubset(std::vector<DataPoint*> &data,
			ros::Time start, ros::Time end);

	/*
	 * Filters the data into separate bagfiles according to the timestamp file at 'filename'.
	 */
	static void filterData(std::string filename);

	/*
	 * Write data to a bagfile. The file to be written to corresponds to 'behaviorName'.
	 */
	static void writeToFile(rosbag::Bag &bag, classification::TrainingData data,
			ros::Time &timeToWrite);
private:

	/*
	 * Get the name of the bagfile contained within 'directory' which includes the given timestamp.
	 */
	static std::string findFile(std::string directory,
			boost::posix_time::ptime timestamp);

	/*
	 * Reads an entire timestamp file and returns a vector of strings that can be passed into
	 * the parseTimestamp method.
	 */
	static std::vector<std::string> readTimestampFile(std::string filename);

	/*
	 * Takes a single timestamp (3 lines: behavior, prompt, correct/incorrect)
	 * and modifies start and end times accordingly.
	 */
	static void parseTimestamp(std::string timestamp, ros::Time &start,
			ros::Time &end, std::string &behaviorName);

	/*
	 * Extract a whole pose from a vector of transforms.
	 */
	static PoseData *extractPose(
			const std::vector<geometry_msgs::TransformStamped>);

	/*
	 * Get the index in 'paths' that corresponds to the file including the time at 'timestamp'.
	 */
	static int getFileIndex(std::vector<boost::filesystem::path>& paths, boost::posix_time::ptime timestamp);
};

}

#endif /* DATALOADER_H_ */
