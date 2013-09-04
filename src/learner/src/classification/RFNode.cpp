#include <nao_autism_messages/PoseClassification.h>

#include <vector>
#include <map>
#include <exception>
#include <iostream>
#include <fstream>

#include <ros/ros.h>
#include <tf/tfMessage.h>
#include <geometry_msgs/TransformStamped.h>

#include <PoseData.h>

#include <std_msgs/Int32.h>

#include <iostream>
#include <fstream>

#include <vector>

#include <opencv/cv.h>
#include <opencv/ml.h>

void tfCallback(const tf::tfMessage msg);

class TempPoseData
{

public:
	TempPoseData()
	{
		nullAll();
	}

	geometry_msgs::TransformStamped* head;
	geometry_msgs::TransformStamped* neck;
	geometry_msgs::TransformStamped* torso;
	geometry_msgs::TransformStamped* left_shoulder;
	geometry_msgs::TransformStamped* left_elbow;
	geometry_msgs::TransformStamped* left_hand;
	geometry_msgs::TransformStamped* right_shoulder;
	geometry_msgs::TransformStamped* right_elbow;
	geometry_msgs::TransformStamped* right_hand;
	geometry_msgs::TransformStamped* left_hip;
	geometry_msgs::TransformStamped* left_knee;
	geometry_msgs::TransformStamped* left_foot;
	geometry_msgs::TransformStamped* right_hip;
	geometry_msgs::TransformStamped* right_knee;
	geometry_msgs::TransformStamped* right_foot;

	bool valid() const
	{
		return head && neck && torso && left_shoulder && left_elbow && left_hand &&
				right_shoulder && right_elbow && right_hand && left_hip && left_knee &&
				left_foot && right_hip && right_knee && right_foot;
	}

	void clear()
	{
		deleteAll();
		nullAll();
	}

private:
	inline void nullAll()
	{
		head = NULL;
		neck = NULL;
		torso = NULL;
		left_shoulder = NULL;
		left_elbow = NULL;
		left_hand = NULL;
		right_shoulder = NULL;
		right_elbow = NULL;
		right_hand = NULL;
		left_hip = NULL;
		left_knee = NULL;
		left_foot = NULL;
		right_hip = NULL;
		right_knee = NULL;
		right_foot = NULL;
	}

	inline void deleteAll()
	{
		if (head){
			delete head;
		}

		if (neck){
			delete neck;
		}

		if (torso){
			delete torso;
		}

		if (left_shoulder){
			delete left_shoulder;
		}

		if (left_elbow){
			delete left_elbow;
		}

		if (left_hand){
			delete left_hand;
		}

		if (right_shoulder){
			delete right_shoulder;
		}

		if (right_elbow){
			delete right_elbow;
		}

		if (right_hand){
			delete right_hand;
		}

		if (left_hip){
			delete left_hip;
		}

		if (left_knee){
			delete left_knee;
		}

		if (left_foot){
			delete left_foot;
		}

		if (right_hip){
			delete right_hip;
		}

		if (right_knee){
			delete right_knee;
		}

		if (right_foot){
			delete right_foot;
		}
	}
};

ros::Subscriber _tf_subscriber;
ros::Publisher _classification_publisher;

std::map<int, TempPoseData> pose_map;

CvRTrees* _rTrees;

struct Instance
{
	float classification;
	std::vector<float> data;
};

#define NUMBER_OF_ATTRIBUTES 36 //Hard coded, for now (maybe? idk)

char* getArgValue(int numberOfArguments, char** arguements, char* key)
{
	for (int i=0;i<numberOfArguments;i++){
		char* current = arguements[i];

		if (std::strcmp(current, key) == 0){
			int valIndex = i+1;

			if (valIndex < numberOfArguments){
				return arguements[valIndex];
			}
		}
	}

	return NULL;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "rf_node");

	int forestSize = 1000;
	int attributeSample = 16; //three suggested values 0.5*sqrt(n), sqrt(n), 2*sqrt(n) where n is equal to number of attributes
	float forestAccuracy = 0.01f;

	char* fileName = getArgValue(argc, argv, "--file-location");

	if (fileName == NULL){
		std::cout << "No --file-location argument has been supplied. Exiting.\n";

		return 1;
	}

	char* forestSizeChar = getArgValue(argc, argv, "--forest-size");

	if (forestSizeChar != NULL){
		forestSize = strtol(forestSizeChar, NULL, 10);

		std::cout << "Using forest size: " << forestSize << std::endl;
	}

	char* attributeSampleChar = getArgValue(argc, argv, "--attribute-sample");

	if (attributeSampleChar != NULL){
		attributeSample = strtol(attributeSampleChar, NULL, 10);

		std::cout << "Using attribute sample (m): " << attributeSample << std::endl;
	}

	char* forestAccuracyChar = getArgValue(argc, argv, "--forest-accuracy");

	if (forestAccuracyChar != NULL){
		forestAccuracy = atof(forestAccuracyChar);

		std::cout << "Using forest accuracy (OOB error): " << forestAccuracy << std::endl;
	}

	std::cout << "Loading data from: " << fileName << std::endl;

	std::vector<Instance> instances;

	//Load in training data
	std::ifstream ifs;
	ifs.open(fileName, std::ios::in);

	while (!ifs.eof()){
		char c;

		bool eof;

		do{
			ifs >> c;
			eof = ifs.eof();
		}while ((c == ' ' || c == '\n') && !eof);

		if (eof)
			break;

		//New instance
		if (c == '+' || c == '-'){
			ifs.unget();

			Instance newInstance;
			ifs >> newInstance.classification;

			for (int i=0;i<NUMBER_OF_ATTRIBUTES;i++){
				int pos;
				ifs >> pos;

				ifs >> c;
				if (c == ':'){
					float val;

					ifs >> val;
					newInstance.data.push_back(val);
				}
			}

			instances.push_back(newInstance);
		}
	}

	printf("File read.\n");

	ifs.close();

	cv::Mat instancesMatrix(instances.size(), NUMBER_OF_ATTRIBUTES, CV_32F);
	cv::Mat classificationMatrix(instances.size(), 1, CV_32F);

	for (int row=0;row<instances.size();row++){
		Instance& current = instances[row];

		for (int column=0;column<current.data.size();column++){
			instancesMatrix.at<float>(row, column) = current.data[column];
		}

		classificationMatrix.at<float>(row, 0) = current.classification;
	}


    cv::Mat var_type(NUMBER_OF_ATTRIBUTES + 1, 1, CV_8U );
    var_type.setTo(cv::Scalar(CV_VAR_NUMERICAL) );
    var_type.at<uchar>(NUMBER_OF_ATTRIBUTES, 0) = CV_VAR_CATEGORICAL;

	std::cout << "Loaded into OpenCV format.\nSetting up Random Forest.\n";

	float priors[] = {1.0f, 1.0f, 1.0f, 1.0f, 1.0f};

	//TODO: Perform some form of grid search to find optimal depth and min sample count.

	CvRTParams params = CvRTParams(25, // max depth
	                                       5, // min sample count
	                                       0, // regression accuracy: N/A here
	                                       false, // compute surrogate split, no missing data
	                                       5, // max number of categories (use sub-optimal algorithm for larger numbers)
	                                       priors, // the array of priors
	                                       true,  // calculate variable importance
	                                       attributeSample,       // number of variables randomly selected at node and used to find the best split(s).
	                                       forestSize,	 // max number of trees in the forest
	                                       forestAccuracy,				// forrest accuracy
	                                       CV_TERMCRIT_ITER |	CV_TERMCRIT_EPS // termination cirteria
	                              );

    _rTrees = new CvRTrees;
    _rTrees->train(instancesMatrix, CV_ROW_SAMPLE, classificationMatrix,
                 cv::Mat(), cv::Mat(), var_type, cv::Mat(), params);
    std::cout << "Training error: " << _rTrees->get_train_error() << std::endl;
    float val = 0;
    cv::Mat varImportance = _rTrees->get_var_importance();

    int nextCol = 0;
    for (int col=0;col<varImportance.cols;col=nextCol){
    	nextCol = col + 1;
    	std::cout << "Variable importance of attribute " << nextCol << " : " << (varImportance.at<float>(0, col)*100) << "%"<< std::endl;
    }

    std::cout << "Setting up Node.\n";

    ros::NodeHandle _nh;
	_tf_subscriber = _nh.subscribe("/tf", 15, tfCallback);
	_classification_publisher = _nh.advertise<nao_autism_messages::PoseClassification>(
			"/classification", 1);

    std::cout << "Setup complete, spinning.\n";

    ros::spin();

	return 0;
}

void tfCallback(const tf::tfMessage msg)
{
	if (msg.transforms.size() > 0){
		geometry_msgs::TransformStamped tsn = msg.transforms[0];

		std::string str(tsn.child_frame_id);
		for (int i=0;i<str.length();i++){
			char curr = str[i];

			if (curr >= '0' && curr <= '9'){
				std::string newStr(str.substr(i));

				int val = strtol(newStr.c_str(), NULL, 10);

				if (pose_map.find(val) == pose_map.end()){
					pose_map.insert(std::pair<int, TempPoseData>(val, TempPoseData()));
				}

				geometry_msgs::TransformStamped* ts = new geometry_msgs::TransformStamped(tsn);
				TempPoseData& point = pose_map.at(val);

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

					if (point.valid()){
						PoseData poseData;

						poseData.head = *point.head;
						poseData.neck = *point.neck;
						poseData.torso = *point.torso;
						poseData.left_shoulder = *point.left_shoulder;
						poseData.left_elbow = *point.left_elbow;
						poseData.left_hand = *point.left_hand;
						poseData.right_shoulder = *point.right_shoulder;
						poseData.right_elbow = *point.right_elbow;
						poseData.right_hand = *point.right_hand;
						poseData.left_hip = *point.left_hip;
						poseData.left_knee = *point.left_knee;
						poseData.left_foot = *point.left_foot;
						poseData.right_hip = *point.right_hip;
						poseData.right_knee = *point.right_knee;
						poseData.right_foot = *point.right_foot;

						//Convert the pose data into matrix notation
						cv::Mat data(1, NUMBER_OF_ATTRIBUTES, CV_32F);

						data.at<float>(0, 0) = point.head->transform.rotation.x;
						data.at<float>(0, 1) = point.head->transform.rotation.y;
						data.at<float>(0, 2) = point.head->transform.rotation.z;
						data.at<float>(0, 3) = point.head->transform.rotation.w;

						data.at<float>(0, 4) = point.neck->transform.rotation.x;
						data.at<float>(0, 5) = point.neck->transform.rotation.y;
						data.at<float>(0, 6) = point.neck->transform.rotation.z;
						data.at<float>(0, 7) = point.neck->transform.rotation.w;

						data.at<float>(0, 8) = point.torso->transform.rotation.x;
						data.at<float>(0, 9) = point.torso->transform.rotation.y;
						data.at<float>(0, 10) = point.torso->transform.rotation.z;
						data.at<float>(0, 11) = point.torso->transform.rotation.w;

						data.at<float>(0, 12) = point.left_shoulder->transform.rotation.x;
						data.at<float>(0, 13) = point.left_shoulder->transform.rotation.y;
						data.at<float>(0, 14) = point.left_shoulder->transform.rotation.z;
						data.at<float>(0, 15) = point.left_shoulder->transform.rotation.w;

						data.at<float>(0, 16) = point.left_elbow->transform.rotation.x;
						data.at<float>(0, 17) = point.left_elbow->transform.rotation.y;
						data.at<float>(0, 18) = point.left_elbow->transform.rotation.z;
						data.at<float>(0, 19) = point.left_elbow->transform.rotation.w;

						data.at<float>(0, 20) = point.left_hand->transform.rotation.x;
						data.at<float>(0, 21) = point.left_hand->transform.rotation.y;
						data.at<float>(0, 22) = point.left_hand->transform.rotation.z;
						data.at<float>(0, 23) = point.left_hand->transform.rotation.w;

						data.at<float>(0, 24) = point.right_shoulder->transform.rotation.x;
						data.at<float>(0, 25) = point.right_shoulder->transform.rotation.y;
						data.at<float>(0, 26) = point.right_shoulder->transform.rotation.z;
						data.at<float>(0, 27) = point.right_shoulder->transform.rotation.w;

						data.at<float>(0, 28) = point.right_elbow->transform.rotation.x;
						data.at<float>(0, 29) = point.right_elbow->transform.rotation.y;
						data.at<float>(0, 30) = point.right_elbow->transform.rotation.z;
						data.at<float>(0, 31) = point.right_elbow->transform.rotation.w;

						data.at<float>(0, 32) = point.right_hand->transform.rotation.x;
						data.at<float>(0, 33) = point.right_hand->transform.rotation.y;
						data.at<float>(0, 34) = point.right_hand->transform.rotation.z;
						data.at<float>(0, 35) = point.right_hand->transform.rotation.w;

//						data.at<float>(0, 36) = point.left_hip->transform.rotation.x;
//						data.at<float>(0, 37) = point.left_hip->transform.rotation.y;
//						data.at<float>(0, 38) = point.left_hip->transform.rotation.z;
//						data.at<float>(0, 39) = point.left_hip->transform.rotation.w;
//
//						data.at<float>(0, 40) = point.left_knee->transform.rotation.x;
//						data.at<float>(0, 41) = point.left_knee->transform.rotation.y;
//						data.at<float>(0, 42) = point.left_knee->transform.rotation.z;
//						data.at<float>(0, 43) = point.left_knee->transform.rotation.w;
//
//						data.at<float>(0, 44) = point.left_foot->transform.rotation.x;
//						data.at<float>(0, 45) = point.left_foot->transform.rotation.y;
//						data.at<float>(0, 46) = point.left_foot->transform.rotation.z;
//						data.at<float>(0, 47) = point.left_foot->transform.rotation.w;
//
//						data.at<float>(0, 48) = point.right_hip->transform.rotation.x;
//						data.at<float>(0, 49) = point.right_hip->transform.rotation.y;
//						data.at<float>(0, 50) = point.right_hip->transform.rotation.z;
//						data.at<float>(0, 51) = point.right_hip->transform.rotation.w;
//
//						data.at<float>(0, 52) = point.right_knee->transform.rotation.x;
//						data.at<float>(0, 53) = point.right_knee->transform.rotation.y;
//						data.at<float>(0, 54) = point.right_knee->transform.rotation.z;
//						data.at<float>(0, 55) = point.right_knee->transform.rotation.w;
//
//						data.at<float>(0, 56) = point.right_foot->transform.rotation.x;
//						data.at<float>(0, 57) = point.right_foot->transform.rotation.y;
//						data.at<float>(0, 58) = point.right_foot->transform.rotation.z;
//						data.at<float>(0, 59) = point.right_foot->transform.rotation.w;

						//Classify point
						float thisClass = _rTrees->predict(data);

						//Create message and send classification
						nao_autism_messages::PoseClassification pc;

						pc.user_number = val;
						pc.classification = static_cast<int>(thisClass);
						pc.pose_data = poseData.getJoints();

						_classification_publisher.publish(pc);
					}

					point.clear();
				}
			}
		}
	}
}
