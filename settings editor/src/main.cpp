#include <QApplication>
#include <QPlastiqueStyle>

#include <QWidget>

#include <Window.h>

int main(int argc, char** argv)
{
	QApplication app(argc, argv);
	app.setStyle(new QPlastiqueStyle);

	Window window;

	return app.exec();
}
