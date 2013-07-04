/*
 * dataLoader.h
 *
 *  Created on: 3 Jul 2013
 *      Author: tristan
 */

#ifndef DATALOADER_H_
#define DATALOADER_H_

#include <poseData.h>
#include <vector>
#include <string>

#include <ros/ros.h>

class DataLoader
{
public:
	static std::vector<PoseData> loadData(std::string filename);

private:
	static PoseData extractPose(const std::vector<geometry_msgs::TransformStamped>);
};

#endif /* DATALOADER_H_ */
