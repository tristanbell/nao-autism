#include <TextInputDialog.h>

#include <QLabel>
#include <QGridLayout>

void TextInputDialog::init(QString& title, QString& labelName)
{
	setWindowTitle(title);

	QGridLayout* layout = new QGridLayout;
	setLayout(layout);

	_label = new QLabel(labelName);
	layout->addWidget(_label, 0, 0);

	_lineEdit = new QLineEdit;
	layout->addWidget(_lineEdit, 1, 0);

	QGridLayout* btnLayout = new QGridLayout;

	_submitBtn = new QPushButton("Submit");
	btnLayout->addWidget(_submitBtn, 0, 0);

	_cancelBtn = new QPushButton("Cancel");
	btnLayout->addWidget(_cancelBtn, 0, 1);

	layout->addLayout(btnLayout, 2, 0);

	QObject::connect(_submitBtn, SIGNAL(clicked()),
			this, SLOT(onSubmitButtonClicked()));
	QObject::connect(_cancelBtn, SIGNAL(clicked()),
			this, SLOT(onCancelButtonClicked()));
}

int TextInputDialog::exec()
{
	//Reset state
	_result = UNKNOWN;

	if (!_noClear)
		_lineEdit->clear();

	return QDialog::exec();
}

TextInputDialog::Result TextInputDialog::getResult() const
{
	return _result;
}

void TextInputDialog::setLabelName(QString& text)
{
	_label->setText(text);
}

void TextInputDialog::setLabelName(std::string& text)
{
	QString qStr = QString::fromStdString(text);

	setLabelName(qStr);
}

void TextInputDialog::setTitle(QString& text)
{
	setWindowTitle(text);
}

void TextInputDialog::setTitle(std::string& text)
{
	QString qStr = QString::fromStdString(text);

	setTitle(qStr);
}

void TextInputDialog::setInput(QString& input)
{
	_lineEdit->setText(input);

	_noClear = true;
}

void TextInputDialog::setInput(std::string& input)
{
	QString qStr = QString::fromStdString(input);

	setInput(qStr);
}

QString TextInputDialog::getQInput() const
{
	return _lineEdit->displayText();
}

std::string TextInputDialog::getInput() const
{
	QString str = _lineEdit->displayText();

	return str.toStdString();
}

void TextInputDialog::onSubmitButtonClicked()
{
	_result = CREATED;
	_noClear = false;

	close();
}

void TextInputDialog::onCancelButtonClicked()
{
	_result = CLOSED;
	_noClear = false;

	close();
}
