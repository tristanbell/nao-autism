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
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>

class TextInputDialog : public QDialog
{

	Q_OBJECT

public:
	enum Result{

		CREATED, CLOSED, UNKNOWN

	};

	TextInputDialog() : QDialog()
	{
		QString title = "";
		QString lblName = "";

		init(title, lblName);
	}

	TextInputDialog(QString& title, QString& labelName) : QDialog()
	{
		init(title, labelName);
	}

	virtual int exec();

	Result getResult() const;

	void setLabelName(QString& text);
	void setLabelName(std::string& text);

	void setTitle(QString& text);
	void setTitle(std::string& text);

	void setInput(QString& input);
	void setInput(std::string& input);

	QString getQInput() const;
	std::string getInput() const;

private:
	bool _noClear;

	Result _result;

	QLabel* _label;
	QLineEdit* _lineEdit;

	QPushButton* _submitBtn;
	QPushButton* _cancelBtn;

	void init(QString& title, QString& labelName);

private slots:
	void onSubmitButtonClicked();
	void onCancelButtonClicked();

};

#endif /* TEXTINPUTDIALOG_H_ */
