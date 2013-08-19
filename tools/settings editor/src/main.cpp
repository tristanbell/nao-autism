#include <QApplication>
#include <QPlastiqueStyle>

#include <QWidget>

#include <Window.h>
#include <json/json.h>
#include <fstream>

#include <boost/shared_ptr.hpp>

Json::Value loadJson();

int main(int argc, char** argv)
{
	QApplication app(argc, argv);
	app.setStyle(new QPlastiqueStyle);

	boost::shared_ptr<Model> modelPtr(new Model);
	boost::shared_ptr<Controller> controllerPtr(new Controller(modelPtr.get()));

	Window window(controllerPtr, modelPtr);

	//modelPtr.get()->loadData(json);

	return app.exec();
}
