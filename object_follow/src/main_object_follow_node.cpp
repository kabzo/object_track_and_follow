
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "FollowControll.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "object_follow");
    ROS_INFO("Started object_follow Node.");

    FollowControll * followControll = new FollowControll();

    followControll->follow();

    delete followControll;
}

