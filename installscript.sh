sudo apt-get install ros-groovy-openni-launch ros-groovy-openni-tracker ros-groovy-pocketsphinx libsvm3 libsvm-dev libsvm-tools
source /opt/ros/groovy/setup.bash
roscd pocketsphinx/nodes
sudo chmod +x recognizer.py

cd ~
git clone https://github.com/tristanbell/nao-autism.git
cd nao-autism
catkin_make
cmake tools/settings\ editor/CMakeLists.txt
make -C tools/settings\ editor

mkdir ~/nao
cd ~/nao

echo "- git:
   uri: https://github.com/ahornung/humanoid_msgs
   local-name: humanoid_msgs
- git:
   uri: https://github.com/ahornung/nao_robot
   local-name: nao_robot
- git:
   uri: https://github.com/ahornung/nao_common
   local-name: nao_common" > rosinstall.txt

rosinstall . ~/nao-autism/devel rosinstall.txt
source setup.bash
rosdep install humanoid_msgs nao_robot nao_common
rosmake humanoid_msgs nao_robot nao_common
