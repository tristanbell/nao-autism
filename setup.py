#!/usr/bin/env python

import sys

from subprocess import call

def writeRunFile():
	f = open("run.sh", "w")

	f.write("source nao-autism/devel/setup.bash\nexport ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:`pwd`\n")

	f.flush()
	f.close()

def writeRosInstall():
	f = open("rosinstall.txt", "w")

	f.write("- git:\n   uri: https://github.com/ahornung/humanoid_msgs\n   local-name: stacks/humanoid_msgs\n")
	f.write("- git:\n   uri: https://github.com/ahornung/nao_robot\n   local-name: stacks/nao_robot\n")
	f.write("- git:\n   uri: https://github.com/ahornung/nao_common\n   local-name: stacks/nao_common\n")
	f.write("- git:\n   uri: https://github.com/tristanbell/nao-autism\n   local-name: nao-autism\n")

	f.flush()
	f.close()

def installRepo():
	print("Pulling repositories")
	call("rosinstall . /opt/ros/groovy/ rosinstall.txt", shell=True)

	print("Running rosdep install")
	call("rosdep install humanoid_msgs nao_robot nao_common", shell=True)

	print("Running rosmake on humanoid_msgs, nao_robot and nao_common")
	call("rosmake humanoid_msgs nao_robot nao_common", shell=True)

	print("Running catkin-make on nao-autism")
	call("catkin_make -C nao-autism", shell=True)

if __name__ == "__main__":
	print("Generating run.sh file")
	writeRunFile()

	print("Generating rosinstall.txt file")
	writeRosInstall()

	print("Installing repositories")
	installRepo()

	print("Setup complete")
