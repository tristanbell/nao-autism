int32 user_number
int32 classification

# A full set of transforms representing each joint positions of the user
geometry_msgs/TransformStamped[] pose_data
# Type of machine learning method (svm, knn, rdf, etc)
string source