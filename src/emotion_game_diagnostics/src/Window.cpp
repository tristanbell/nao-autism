#include <Window.h>

#include <QGridLayout>

void Window::init()
{
	setWindowTitle("Nao Diagnostics");

	QGridLayout* layout = new QGridLayout;

	_generalInformationBox = new GeneralInformationBox;
	layout->addWidget(_generalInformationBox, 0, 0, 2, 1);

	_executionControlBox = new ExecutionControlBox;
	layout->addWidget(_executionControlBox, 2, 0, 1, 2);

	_nodeBox = new NodeBox;
	layout->addWidget(_nodeBox, 0, 1, 2, 1);

	setLayout(layout);

	setBaseSize(100, 100);
	setVisible(true);
}
