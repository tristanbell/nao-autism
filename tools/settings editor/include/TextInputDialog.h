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

	Q_OBJECT

public:
	enum Result{

		CREATED, CLOSED, UNKNOWN

	};

	TextInputDialog(QString& title, QString& labelName) : QDialog()
	{
		init(title, labelName);
	}

	virtual int exec();

	Result getResult() const;
	QString getQInput() const;
	std::string getInput() const;

private:
	Result _result;

	QLineEdit* _lineEdit;

	QPushButton* _submitBtn;
	QPushButton* _cancelBtn;

	void init(QString& title, QString& labelName);

private slots:
	void onSubmitButtonClicked();
	void onCancelButtonClicked();

};

#endif /* TEXTINPUTDIALOG_H_ */
