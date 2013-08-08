#include <TextInputDialog.h>

#include <QLabel>
#include <QGridLayout>

void TextInputDialog::init(QString& title, QString& labelName)
{
	setWindowTitle(title);

	QGridLayout* layout = new QGridLayout;
	setLayout(layout);

	QLabel* lbl = new QLabel(labelName);
	layout->addWidget(lbl, 0, 0);

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
	_result = UNKNOWN;

	return QDialog::exec();
}

TextInputDialog::Result TextInputDialog::getResult() const
{
	return _result;
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

	close();
}

void TextInputDialog::onCancelButtonClicked()
{
	_result = CLOSED;

	close();
}
