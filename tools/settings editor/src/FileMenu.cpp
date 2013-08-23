#include <FileMenu.h>

#include <iostream>

#include <QFileDialog>
#include <QErrorMessage>
#include <QMessageBox>

#define FILE_FILTER "JSON Data (*.json)"

void FileMenu::init(){
	_newAction = new QAction("New", this);
	addAction(_newAction);

	_openAction = new QAction("Open", this);
	addAction(_openAction);

	_saveAction = new QAction("Save", this);
	_saveAction->setEnabled(false);
	addAction(_saveAction);

	_saveAsAction = new QAction("Save as", this);
	addAction(_saveAsAction);

	QObject::connect(_newAction, SIGNAL(triggered()),
			this, SLOT(newTriggered()));
	QObject::connect(_openAction, SIGNAL(triggered()),
			this, SLOT(openTriggered()));
	QObject::connect(_saveAction, SIGNAL(triggered()),
			this, SLOT(saveTriggered()));
	QObject::connect(_saveAsAction, SIGNAL(triggered()),
			this, SLOT(saveAsTriggered()));
}

void FileMenu::newTriggered()
{
	_saveAction->setEnabled(false);

	emit onNewRequested();
}

void FileMenu::openTriggered()
{
	QString name = QFileDialog::getOpenFileName(this, "Open file", ".", FILE_FILTER);

	std::string location = name.toStdString();
	emit onOpenRequested(location);
}

void FileMenu::saveTriggered()
{
	emit onSaveRequested();
}

void FileMenu::saveAsTriggered()
{
	QString name = QFileDialog::getSaveFileName(this, "Save file as", ".", FILE_FILTER);

	std::string location = name.toStdString();
	emit onSaveAsRequested(location);
}

void FileMenu::onSuccessfulOpen(const std::string& location)
{
	QString qMessage = QString::fromStdString("Successfully opened the following file: " + location);

	QMessageBox::information(this, "Successfully opened", qMessage);

	_saveAction->setEnabled(true);
}

void FileMenu::onUnsuccessfulOpen(const std::string& reason)
{
	QString qReason = QString::fromStdString(reason);

	QMessageBox::critical(this, "Error", qReason);
}

void FileMenu::onSuccessfulSave(const std::string& location)
{
	QString qMessage = QString::fromStdString("Successfully saved to the following file: " + location);

	QMessageBox::information(this, "Successfully saved", qMessage);

	_saveAction->setEnabled(true);
}

void FileMenu::onUnsuccessfulSave(const std::string& reason)
{
	QString qReason = QString::fromStdString(reason);

	QMessageBox::critical(this, "Error", qReason);
}
