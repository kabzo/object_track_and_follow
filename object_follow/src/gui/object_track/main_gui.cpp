#include <ros/ros.h>

#include "object_follow_gui.h"
#include <QApplication>
#include <signal.h>
#include <QtGui/QApplication>

void termination_handler(int signum){
    QApplication::quit();
}


int main(int argc,char *argv[])
{
    ros::init(argc, argv, "object_follow_gui");
    ROS_INFO("Started object_follow_gui Node.");

//    QApplication a(argc, argv);
//    object_follow_gui w;
//    w.show();

    ros::AsyncSpinner spinner(1);
    struct sigaction action;

    /* Set up the structure to specify the action */
    action.sa_handler = termination_handler;
    sigemptyset(&action.sa_mask);
    action.sa_flags = 0;
    sigaction(SIGINT, &action, NULL);

    QApplication * app = new QApplication(argc, argv);
    object_follow_gui * gui = new object_follow_gui();
    gui->setMinimumSize(640,360);
    gui->show();

    spinner.start();
    app->exec();
    spinner.stop();

    delete gui;
    delete app;

    return 0;

//    return a.exec();
}
