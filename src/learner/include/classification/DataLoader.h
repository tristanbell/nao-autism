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
#include <vector>
#include <string>

#include <boost/date_time/posix_time/posix_time.hpp>

#include <ros/ros.h>

namespace classification{

class DataLoader
{
public:
	static std::vector<DataPoint*> loadData(std::string filename);
	static void parseTimestamp(std::string timestamp, ros::Time &start, ros::Time &end, std::string &behaviorName);
	static std::vector<DataPoint*> getDataSubset(std::vector<DataPoint*> &data, ros::Time start, ros::Time end);

	/*
	 * Get the name of the bagfile contained within a directory which includes the given timestamp.
	 */
	static std::string findFile(std::string directory, boost::posix_time::ptime timestamp);

private:
	static PoseData *extractPose(const std::vector<geometry_msgs::TransformStamped>);
};

}

#endif /* DATALOADER_H_ */
