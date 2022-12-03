
#include <QtGui>
#include <QApplication>
#include "../include/gnss_ros_gui/main_window.hpp"


int main(int argc, char **argv) {
    
    QApplication app(argc, argv);
    gnss_ros_gui::MainWindow w(argc,argv);
    w.show();
    app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));
    int result = app.exec();

	return result;
}
