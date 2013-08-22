/*
 * NodeBox.h
 *
 *  Created on: 21 Aug 2013
 *      Author: alex
 */

#ifndef NODEBOX_H_
#define NODEBOX_H_

#include <QGroupBox>

#include <QLabel>
#include <QGridLayout>

#include <boost/thread.hpp>

class NodeBox : public QGroupBox
{

	Q_OBJECT

public:
	NodeBox() : QGroupBox()
	{
		init();
	}

private:
	boost::thread _nodeCheckingThread;

	struct NodeInformation{
		NodeInformation() : _running(false),
							_attemptedToRun(false)
		{ }

		std::string _name;

		bool _running;

		std::string _startCommand;
		bool _attemptedToRun;
		time_t _runAttemptTime;
	};

	struct NodeInformationGroup{
		QGridLayout* _layout;

		QLabel* _nameLabel;
		QLabel* _statusLabel;
	};

	std::vector<NodeInformation> _nodeInformationVector;
	std::map<std::string, NodeInformationGroup> _nodeMap;

	void init();

	void run();

private slots:
	void onNodeInformationRefresh();

signals:
	void nodeInformationRefresh();

};


#endif /* NODEBOX_H_ */
