#include "drone_control.h"
#include <QApplication>
#include <ros/ros.h>

int main(int argc, char *argv[])
{

    ros::init(argc, argv, "main_drone_control_node");
    ROS_INFO("Started main_drone_control_node Node.");

    ros::AsyncSpinner spinner(1);

    QApplication * a = new QApplication(argc, argv);
    Drone_control *w = new Drone_control();
    w->show();

    spinner.start();
    a->exec();
    spinner.stop();
    return 0;
}
