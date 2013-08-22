#include <NodeBox.h>

#include <QString>

#include <iostream>

#include <ros/ros.h>

#include <string>
#include <vector>

void NodeBox::init()
{
	setTitle("Nodes statuses");

	QGridLayout* layout = new QGridLayout;

	//Add required nodes to nodes information vector
	NodeInformation openniTrackerNode;
	openniTrackerNode._name = "/openni_tracker";
	openniTrackerNode._startCommand = "rosrun openni_tracker openni_tracker &";

	_nodeInformationVector.push_back(openniTrackerNode);

	NodeInformation naoBehaviorsNode;
	naoBehaviorsNode._name = "/nao_behaviors";
	naoBehaviorsNode._startCommand = "rosrun nao_driver nao_behaviors.py --pip=$NAO_IP";

	_nodeInformationVector.push_back(naoBehaviorsNode);

	NodeInformation naoSpeechNode;
	naoSpeechNode._name = "/nao_speech";
	naoSpeechNode._startCommand = "rosrun nao_driver nao_speech.py --pip=$NAO_IP";

	_nodeInformationVector.push_back(naoSpeechNode);

	int rowNumber = 0;
	for (int i=0;i<_nodeInformationVector.size();i++){
		NodeInformation& current = _nodeInformationVector[i];

		NodeInformationGroup grp;
		grp._layout = new QGridLayout;
		grp._nameLabel = new QLabel(QString::fromStdString(current._name));
		grp._statusLabel = new QLabel("Unknown.");

		_nodeMap.insert(std::pair<std::string, NodeInformationGroup>(current._name, grp));

		QLabel* nameLabel = new QLabel("Node name:");
		QLabel* statusLabel = new QLabel("Node status:");

		grp._layout->addWidget(nameLabel, 0, 0);
		grp._layout->addWidget(grp._nameLabel, 0, 1);
		grp._layout->addWidget(statusLabel, 1, 0);
		grp._layout->addWidget(grp._statusLabel, 1, 1);

		layout->addItem(grp._layout, rowNumber, 0);
		rowNumber++;
	}

	setLayout(layout);

	//Start thread for checking running nodes
	_nodeCheckingThread = boost::thread(&NodeBox::run, this);
	//_nodeCheckingThread.start_thread();

	QObject::connect(this, SIGNAL(nodeInformationRefresh()),
			this, SLOT(onNodeInformationRefresh()));
}

void NodeBox::run()
{
	std::cout << "[NodeBox] - Starting node checking thread.\n";

	while (true){
		//First, set all nodes to not running
		for (int i=0;i<_nodeInformationVector.size();i++){
			NodeInformation& current = _nodeInformationVector[i];

			current._running = false;
		}

		std::vector<std::string> nodeList;
		if (ros::master::getNodes(nodeList)){
			//We will now check if each node is running, if it is not we will attempt to start it
			for (int i=0;i<nodeList.size();i++){
				std::string& currentNode = nodeList[i];

				for (int i=0;i<_nodeInformationVector.size();i++){
					NodeInformation& current = _nodeInformationVector[i];

					if (current._name == currentNode){
						current._running = true;
					}
				}
			}

			//Now we know what nodes are running, we will attempt to start ones that aren't
			for (int i=0;i<_nodeInformationVector.size();i++){
				NodeInformation& current = _nodeInformationVector[i];

				if (!current._running){
					//Node is not running, check if recently attempted to launch
					time_t currentTime = time(0);

					if (!current._attemptedToRun || (current._attemptedToRun &&
							(currentTime -current._runAttemptTime) > 60)){
						//Attempt to run node
						std::cout << "Attempting to run: " << current._name << std::endl;

						system(current._startCommand.c_str());

						current._attemptedToRun = true;
						current._runAttemptTime = time(0);
					}
				}
			}
		}

		emit nodeInformationRefresh();

		//Make the thread sleep
		boost::this_thread::sleep(boost::posix_time::milliseconds(100));
	}
}

void NodeBox::onNodeInformationRefresh()
{
	time_t currentTime = time(0);

	for (int i=0;i<_nodeInformationVector.size();i++){
		NodeInformation& current = _nodeInformationVector[i];

		try{
			NodeInformationGroup& grp = _nodeMap[current._name];

			if (current._running){
				grp._statusLabel->setText("Running.");
			}else if (current._attemptedToRun){
				std::stringstream ss;

				ss << " Attempted to run " << (currentTime - current._runAttemptTime) << " seconds ago.";

				grp._statusLabel->setText(QString::fromStdString(ss.str()));
			}else{
				grp._statusLabel->setText("Unknown.");
			}
		}catch(std::out_of_range& ex){ }
	}
}
