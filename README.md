Nao Autism Games
================

This package contains three basic games designed for children with autism. The first two games, Guess the Emotion and Copy the Robot, go hand in hand and are designed to be played one after the other. These two games are based on emotion recognition and replication, as well as turn-taking. The third game is standalone and allows the child to control the robot remotely.

Installation
------------

It is assumed that ROS (the Robot Operating System) is already installed on your computer running Ubuntu Linux. If it is not, you can find installation instructions at http://www.ros.org/wiki/ROS/Installation. The following packages are also required:

+ openni_launch
+ openni_tracker
+ Aldebaran NaoQi SDK
+ Pocketsphinx speech recognition engine
+ nao humanoid_stacks

####OpenNI####
OpenNI packages can be installed via the command line (on Linux) as follows:
```bash
sudo apt-get install ros-groovy-openni-launch 
sudo apt-get install ros-groovy-openni-tracker
```

####NaoQi####
The NaoQi SDK can be downloaded from https://community.aldebaran-robotics.com/resources. Bear in mind that you need to register as a developer with Aldebaran in order to access this. Select NAOQI C++ SDK 1.14.5 Linux (32 bits or 64 bits depending on your operating system), then extract it somewhere on your machine (for our examples we have placed it under /opt/naoqi). Next, we need to define some environment variables that we'll use later when running the games and add the NaoQi lib directory to our PYTHONPATH so ROS can access it. To avoid having to repeat lots of typing, add the following to the end of your `~/.bashrc`:
```bash
# Wherever the NAOqi SDK is installed
export NAOQI_HOME=/opt/naoqi/naoqi-sdk-1.14.5-linux64
export NAOQI_LIBS=$NAOQI_HOME/lib

# Add NAOqi to your python path
export PYTHONPATH=$PYTHONPATH:$NAOQI_LIBS
# Define the library path for running ROS Nao components
export NAOQI_LIBRARY_PATH=$NAOQI_LIBS:$LD_LIBRARY_PATH
```

####Speech Recognition####
Pocketsphinx is used for speech recognition. To install, type the following into the terminal: `sudo apt-get install ros-groovy-pocketsphinx`

The recognizer.py node that this installs is not executable by default, so we need to manually make it executable:
```bash
roscd pocketsphinx/nodes
sudo chmod +x recognizer.py
```

####Nao Autism Games####

#####Easy Installation#####
The simplest way to install is to use the installation script: [link here]. Download and double click the script to run it, and the games will be installed to the correct directory.

#####Manual Installation#####

If you would prefer to install manually, or if the easy installation does not work for any reason, follow these steps to install the games.

First, create a new directory for them and cd into it:
```bash
mkdir ~/nao-autism
cd ~/nao-autism
```
Then make a new file called rosinstall.txt and paste the following inside it:
```bash
- git:
    uri: https://github.com/ahornung/humanoid_msgs
    local-name: stacks/humanoid_msgs
- git:
    uri: https://github.com/ahornung/nao_robot
    local-name: stacks/nao_robot
- git:
    uri: https://github.com/ahornung/nao_common
    local-name: stacks/nao_common
- git:
    uri: https://github.com/tristanbell/nao-autism
    local-name: nao-autism
```
Save and close the file, then run rosinstall to install the relevant packages:
```bash
rosinstall . /opt/ros/groovy rosinstall.txt
rosdep install humanoid_msgs nao_robot nao_common
rosmake humanoid_msgs nao_robot nao_common
catkin_make -C nao-autism
```

Running the Emotion Games
-------------------------

ROS programs are normally run through the command line. In order for our program to run, several others must also be running at the same time. The Kinect must also be running in order for the Copy the Robot game to work.

1. First, plug in the Kinect to a USB port.
2. To start the Kinect, open a terminal window and type `roslaunch openni_launch openni.launch`
3. In a new terminal window, type

```bash
    LD_LIBRARY_PATH=$NAOQI_LIBRARY_PATH
    roslaunch ~/nao-autism/launch/run_nao.launch
```
4. Then, in a new terminal window, make sure you are in the nao-autism directory (`cd ~/nao-autism`) then type `source devel/setup.bash`
5. When running for the first time, you will need a configuration file for the game. To make one, type `tools/settings\ editor/settings_editor` to open the settings editor, then click File > New to generate a basic configuration file. Adjust any of the settings as you see fit, and when you're finished click File > Save As. Save your new configuration file (name it something like "data.json") in the nao-autism folder. Then close the settings editor.
6. Start the emotion recognition games with `roslaunch emotion_game emotion_game.launch data.json` (substitute data.json for whatever you named your configuration file).

Running the Motion Controller
-----------------------------

The Mimicker program allows a user to control the Nao using a Kinect. To run it, do the following:

1. With the Kinect plugged in to a USB port, open a terminal window and type `roslaunch openni_launch openni.launch`
2. In a new terminal window, type

```bash
    source ~/nao-autism/devel/setup.bash
    roslaunch emotion_game kinect_control.launch
```

Once it is running, put your right hand on your head. Once the Kinect recognises someone doing this gesture, it will initialise a virtual 'control box' around this user, giving them control over the robot.

Using the Motion Controller
---------------------------

The kinect motion controller (Mimicker) works in two ways: moving the Nao's arms and making the Nao walk. While stationary, move your arms and the Nao will copy your movements. There are various motions to make it walk:

+ Step forward with either foot to make the Nao walk forward
+ Step backward with either foot to make the Nao walk backward
+ Step to the left with your left foot to make the Nao shuffle to the left
+ Step to the right with your right foot to make the Nao shuffle to the right
+ Turning left or right will make the Nao turn left or right, respectively. This also works while the Nao is walking forwards or backwards

Things to do
------------

+ Adjust pocketsphinx language model to work with english accents (if too difficult, just map 'scared' to 'yes', seems to get that every time)
+ Perhaps tune the SVM to make scared pose more accurate
+ Finish GUI for running the nodes/diagnostics
+ Write documentation
+ Make sure mimicker sits/stands properly
