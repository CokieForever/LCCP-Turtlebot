#include <QApplication>
#include "mainwindow.h"


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "turtlebot_gui");
    QApplication a(argc, argv);
    MainWindow w;
    w.show();

    return a.exec();
}
