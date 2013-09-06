# Repository Overview

In this document you will find a, somewhat, brief overview of the repository structure.

## Main directory

In this folder, you will find the following folders:

- resources/
- documents/
- src/
- tools/

And the following files:

- installscript.sh
- run\_settings\_editor.sh

Firstly, let's consider the use for the folders.

###### Resources folder

The resources folder contains resources for the project, currently it only contains the recordings for the individual emotions. The format for each recording is the 'bag' file format, this is a ROS specific format (for information about this format and the data is contains, please consider the following: http://wiki.ros.org/Bags).

###### Documents folder

The documents folder contains documentation for the project (this file should be located there!).

###### Src folder

The src folder contains all the ROS packages that have been developed for this project, each sub-folder defines a seperate package. A description of each package will be given in the next section.

###### Tools folder

The tools folder contains all the tools that have been developed for this project. These are independant from the packages because they are not dependant on any ROS packages/libraries. Currently there is only one tool developed, which is the settings editor (further information on the settings editor is given in a seperate document).

## Packages

The src folder contains the following packages:

- emotion_game
- emotion\_game\_diagnostics
- learner
- nao\_autism\_gui
- nao\_autism\_messages
- nao\_control
- nao\_msgs
- rosbag\_recorder

Below you will find a quick summary of each package and the nodes it contains.

###### Emotion\_game package

This package contains all the code relating to the emotion game that runs on the Nao. It contains the following nodes:

- start\_game: This node simply starts the emotion game. It must be provided with the location of a settings file (this file should have the json format) as an argument.
- gen\_json: This node generates the default settings file for the emotion game. This node requires only one argument, the location to save the file to.
- mimicker: This node is an extension to the original game, it allows the children to control the robot using the Kinect skeleton tracking.

There are also multiple launch files that will launch all the required nodes, a description of each one and its purpose is given below:

- emotion\_game\_rf.launch: This launch file will launch the emotion game node and its required dependencies (apart from the nodes that translate commands on the computer to actions on the Nao, see: run\_nao.launch). For the pose classifier, it launches the Random Forest node present in the learner package.
- emotion\_game\_svm.launch: This launch file will launch the emotion game node and its required dependencies. For the pose classifier, it launches the Support Vector Machine node present in the learner package.
- kinect\_control.launch: This launch file will launch the required nodes for controller the Nao through the Kinect. It will not run the required nodes for actually performing the commands on the Nao (see: kinect\_nao.launch).
- kinect\_nao.launch: This launch file will launch the required nodes for controlling the Nao through the Kinect. It will not run the required nodes to translate skeleton data into commands (see: kinect\_control.launch).
- run\_nao.launch: Thie launch file will launch the required nodes to translate commands on the computer to actions on the Nao.

###### Emotion\_game\_diagnostics package

This package contains all the code relating to performing diagnostics on the emotion game and the Nao itself, it allows the user to set the lower limit of the battery and the upper limit of the temperature which, when reached, will stop the emotion game, causing the Nao to sit down and prevent any damage. The following nodes are defined in this package:

- run\_gui: This node runs a GUI (graphical user interface) for the user so they can observe (and set limits for) the battery and joint temperature level. They can also issue pause, stop and continue commands to the emotion game through this GUI (this assumes that the emotion game is currently running, if it isn't then action can be taken).
- run\_checker: This node essentially does the same as the node above (apart from being able to control the emotion game), except it doesn't display a GUI. Thus, the values for the limits cannot be changed once it is running.

###### Learner package

This package contains all the code relating to Pose classification. There are also utility nodes present in this package that will take bag data and produce training data from it. This data can then, for example, be run through LibSVM's tools to produce models. The following nodes are present in this package:

- accuracy\_checker: This node launches a GUI to check the accuracy of each classifier. This can either perform accuracy checking on the fly through the use of OpenNI Tracker, or it can be used with rosbag to play-back existing data,

- dataLoader: This node takes a timestamps file (this is produced by the nao\_autism\_gui run\_autism\_gui node) and produces a bunch of bag files containing only the data for each given emotion.

- classification\_listener: This node simply subscribes to the classification topic and prints out the classification of each message, it is useful for quickly checking the accuracy of a given classifier.

- svm\_export: This node takes pair arguments of a bag file and classification (such as: happy.bag 1 sad.bag 2) and produces a training data file.

- knn\_node: This node takes pair arguments of a bag file and classification (such as: happy.bag 1 sad.bag 2, etc) and trains a KNN classifier (see: http://en.wikipedia.org/wiki/K-nearest_neighbors_algorithm), currently the amount of neighbours is hard-coded into the algorithm so no argument can be provided for changing this value. After training the classifier, the node will then listen for skeleton data and publish classifications for each user when possible.

- svm\_node: This node takes a model file and trains a SVM classifier (see: http://en.wikipedia.org/wiki/Support_vector_machine). After training the classifier, the node will then listen for skeleton data and publish classifications for each user when possible.

- rf\_node: This node takes a training data file (the output from the svm\_export node) and trains a RF classifier (see: http://en.wikipedia.org/wiki/Random_forest). After training the classifier, the node will then listen for skeleton data and publish classifications for each user when possible.

This package also contains a resources folder which contains the required files for the classifiers to run when the emotion game is launched.

###### Nao\_autism\_gui

This package contains the code for a 'Wizard of Oz' application for the emotion game. This application, when ran with the launch file, will record all skeleton data for the mimic game. This data can then be used to train the classifiers (after some amount of processing) in the learner package. There is, currently, only one node defined in this package:

- run\_autism\_gui: This node runs the GUI. It must be given the location of the speech data file (the default one is contained in the resources folder) in order to function correctly. 

Note: When the GUI is launched through the launch file then the timestamps and recordings will be saved to the '.ros' folder in the main user directory. This can simply be extraced and placed in the main nao-autism repository directory for processing.

###### Nao\_autism\_messages

This package simply contains the messgaes required for communication between the nodes defined in this project. No expaination for any of the messages will be given as the semantics of the messages should be clear when the code that utilises them is viewed. 

###### Nao\_control

This package contains code that simply bundles speech and behavior execution together. It allows other packages to use a simple interface to perform behaviors and speech by simply creating an instance of the NaoControl class. There are no nodes present in this package, the code present simply defines a library for other pakcages to use.

###### Nao\_msgs

This package was pulled directly from the following ROS package: . This enables the use of speech recognition, individual joint manipulation and much more.

###### Rosbag\_recorder

This package contains code that records tf data that is being published on the /tf topic. This package contains the following node:

- rosbag\_recorder: This node will wait for a Record message to be sent on the topic it is subscribed to. When recieved it will then record all /tf data into the recordings folder (the location this is saved to will be relative to the folder the node was run in, if it is ran through a launch file then the data will be saved to the '.ros' folder in the main user directory).

