#include "mainwindow.h"
#include <QApplication>
#include <ros/ros.h>

int main(int argc, char *argv[])
{

    ros::init(argc, argv, "object_follow_main_guir");
    ROS_INFO("Started object_follow_gui Node.");

    ros::AsyncSpinner spinner(1);

    QApplication a(argc, argv);
    MainWindow w;

    w.show();

    spinner.start();
    a.exec();
    spinner.stop();


    return 0;
}
