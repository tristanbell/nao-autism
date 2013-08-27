Nao Autism Games
================

This package contains three basic games designed for children with autism. The first two games, Guess the Emotion and Copy the Robot, go hand in hand and are designed to be played one after the other. These two games are based on emotion recognition and replication, as well as turn-taking. The third game is standalone and allows the child to control the robot remotely.

Installation
------------

It is assumed that ROS (the Robot Operating System) is already installed on your computer running Ubuntu Linux. If it is not, you can find installation instructions at http://www.ros.org/wiki/ROS/Installation. The following packages are also required:

+ openni_launch
+ openni_tracker
+ Aldebaran NaoQi SDK
+ nao humanoid_stacks
+ nao package by the Birmingham Autonomous Robotics Club

Openni packages can be installed via the command line (on Linux) as follows:
```bash
sudo apt-get install ros-groovy-openni-launch 
sudo apt-get install ros-groovy-openni-tracker
```

The NaoQi SDK can be downloaded from https://community.aldebaran-robotics.com/resources. Bear in mind that you need to be a registered developer in order to access this. Select NAOQI C++ SDK 1.14.5 Linux (32 bits or 64 bits depending on your operating system).

-----nao_components stuff here-----

Running the Games
-----------------

ROS programs are normally run through the command line. In order for our program to run, several others must also be running at the same time.

First, plug in the Kinect to a USB port
To start the Kinect, open a Terminal window and type `roslaunch openni_launch openni.launch`




Things to do
------------

+ Adjust pocketsphinx language model to work with english accents (if too difficult, just map 'scared' to 'yes', seems to get that every time)
+ Perhaps tune the SVM to make scared pose more accurate
+ Finish GUI for running the nodes/diagnostics
+ Write documentation
+ Make sure mimicker sits/stands properly
