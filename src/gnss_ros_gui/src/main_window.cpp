
#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/gnss_ros_gui/main_window.hpp"



namespace gnss_ros_gui {

using namespace Qt;


MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
    : QMainWindow(parent)
    , qnode(argc,argv)
{
    ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
     qnode.init();//初始化节点

}

MainWindow::~MainWindow() {}




}  // namespace gnss_ros_gui

