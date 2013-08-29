ORIGINAL_DIR=`pwd`

sudo apt-get install ros-groovy-openni-launch 
sudo apt-get install ros-groovy-openni-tracker

sudo apt-get install ros-groovy-pocketsphinx
roscd pocketsphinx/nodes
sudo chmod +x recognizer.py

mkdir ~/nao-autism
cd ~/nao-autism
python $ORIGINAL_DIR/setup.py
