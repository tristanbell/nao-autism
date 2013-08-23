/*
 * FileMenu.h
 *
 *  Created on: 8 Aug 2013
 *      Author: rapid
 */

#ifndef FILEMENU_H_
#define FILEMENU_H_

#include <QMenu>
#include <QAction>

class FileMenu : public QMenu
{

	Q_OBJECT

public:
	FileMenu() : QMenu()
	{
		setTitle("File");

		init();
	}

private:
	QAction* _newAction;
	QAction* _openAction;
	QAction* _saveAction;
	QAction* _saveAsAction;

	void init();

private slots:
	void newTriggered();
	void openTriggered();
	void saveTriggered();
	void saveAsTriggered();

	void onSuccessfulOpen(const std::string& location);
	void onUnsuccessfulOpen(const std::string& reason);

	void onSuccessfulSave(const std::string& location);
	void onUnsuccessfulSave(const std::string& reason);

signals:
	void onNewRequested();
	void onOpenRequested(const std::string& location);
	void onSaveRequested();
	void onSaveAsRequested(const std::string& location);

};


#endif /* FILEMENU_H_ */
