/*
 * TextInputDialog.h
 *
 *  Created on: 7 Aug 2013
 *      Author: alex
 */

#ifndef TEXTINPUTDIALOG_H_
#define TEXTINPUTDIALOG_H_

#include <QString>
#include <QDialog>
#include <QLineEdit>
#include <QPushButton>

class TextInputDialog : public QDialog
{

public:
	TextInputDialog(QString& title, QString& labelName) : QDialog()
	{
		init(title, labelName);
	}

	std::string getInput() const;

private:
	QLineEdit* _lineEdit;

	QPushButton* _submitBtn;
	QPushButton* _cancelBtn;

	void init(QString& title, QString& labelName);

};

#endif /* TEXTINPUTDIALOG_H_ */
