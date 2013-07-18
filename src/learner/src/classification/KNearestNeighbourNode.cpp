#include <classification/Learner.h>
#include <classification/KNearestNeighbour.h>
#include <classification/DataLoader.h>
#include <classification/DataStore.h>
#include <classification/PlainDataStore.h>
#include <classification/DataPoint.h>

#include <learner/PoseClassification.h>

#include <vector>
#include <map>
#include <exception>

#include <tf/tfMessage.h>
#include <geometry_msgs/TransformStamped.h>

#include <ros/ros.h>
#include <std_msgs/Int32.h>

//Method prototypes
void tfCallback(const tf::tfMessage msg);

classification::Learner* knn_learner;
ros::Subscriber tf_subscriber;
ros::Publisher classification_publisher;

std::map<int, PoseData> pose_map;

int main(int argc, char** argv)
{
	if ((argc % 2) == 1 && argc > 1){
		ros::init(argc, argv, "knn_node");

		classification::TrainingData classifiedPoints;

		ROS_INFO("Scanning argument(s)");
		for (unsigned int i=1;i<argc;i+=2){
			std::string fileName(argv[i]);
			short classification = strtol(argv[i+1], NULL, 10);

			std::vector<classification::DataPoint*> dataPoints = classification::DataLoader::loadData(fileName);
			for (unsigned int j=0;j<dataPoints.size();j++){
				classification::DataPoint* current = dataPoints[j];

				current->setClassification(classification);
				classifiedPoints.push_back(current);
			}
		}

		ROS_INFO("Finished retrieving data.");

		classification::DataStore* store = new classification::PlainDataStore(classifiedPoints);
		knn_learner = new classification::KNearestNeighbour(store, 5);
		ROS_INFO("Created KNN instance.");

		ROS_INFO("Creating subscriber to /tf");
		ros::NodeHandle nh;

		tf_subscriber = nh.subscribe("/tf", 15, tfCallback);

		ROS_INFO("Creating classification advertiser");
		classification_publisher = nh.advertise<learner::PoseClassification>("/classification", 1);

		ROS_INFO("Setup complete, spinning");
		ros::spin();
	}else{
		ROS_INFO("Invalid arguments, The arguments should be supplied in pairs of <filename> and <classification>");
		ROS_INFO("Where the filename is the name of the bag file containing solely tf transforms and the classification is some short integer.");

		return -1;
	}

	return 0;
}

void tfCallback(const tf::tfMessage msg)
{
	if (msg.transforms.size() > 0){
		geometry_msgs::TransformStamped ts = msg.transforms[0];

		std::string str(ts.child_frame_id);
		for (int i=0;i<str.length();i++){
			char curr = str[i];

			if (curr >= '0' && curr <= '9'){
				std::string newStr(str.substr(i));

				int val = strtol(newStr.c_str(), NULL, 10);

				if (pose_map.find(val) == pose_map.end()){
					pose_map.insert(std::pair<int, PoseData>(val, PoseData()));
				}

				PoseData& point = pose_map.at(val);

				//Long if statement ftw...
				if (str.find("head") != std::string::npos){
					point.head = ts;
				}else if (str.find("neck") != std::string::npos){
					point.neck = ts;
				}else if (str.find("torso") != std::string::npos){
					point.torso = ts;
				}else if (str.find("left_shoulder") != std::string::npos){
					point.left_shoulder = ts;
				}else if (str.find("left_elbow") != std::string::npos){
					point.left_elbow = ts;
				}else if (str.find("left_hand") != std::string::npos){
					point.left_hand = ts;
				}else if (str.find("right_shoulder") != std::string::npos){
					point.right_shoulder = ts;
				}else if (str.find("right_elbow") != std::string::npos){
					point.right_elbow = ts;
				}else if (str.find("right_hand") != std::string::npos){
					point.right_hand = ts;
				}else if (str.find("left_hip") != std::string::npos){
					point.left_hip = ts;
				}else if (str.find("left_knee") != std::string::npos){
					point.left_knee = ts;
				}else if (str.find("left_foot") != std::string::npos){
					point.left_foot = ts;
				}else if (str.find("right_hip") != std::string::npos){
					point.right_hip = ts;
				}else if (str.find("right_knee") != std::string::npos){
					point.right_knee = ts;
				}else if (str.find("right_foot") != std::string::npos){
					point.right_foot = ts;

					//Classify point
					classification::PoseDataPoint* pdp = new classification::PoseDataPoint(point);
					int classification = knn_learner->classify(pdp);
					delete pdp;

					//Create message and send classification
					learner::PoseClassification pc;

					pc.user_number = val;
					pc.classification = classification;

					classification_publisher.publish(pc);
				}
			}
		}
	}
}
