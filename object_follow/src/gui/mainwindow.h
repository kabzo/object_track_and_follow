#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "ui_mainwindow.h"


#include <QMainWindow>

#include<ros/ros.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <tld_msgs/Target.h>
#include <tld_msgs/BoundingBox.h>
#include <std_msgs/Char.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <opencv/cv.h>

#include "object_follow/controller_cmd.h"
#include <QKeyEvent>
#include <image_transport/image_transport.h>


class MainWindow : public QMainWindow, public Ui_MainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    void set_nodeHandler(ros::NodeHandle &n);
    static pthread_mutex_t send_CS;
private:
    ros::NodeHandle _n;

    image_transport::ImageTransport it_;
    image_transport::Subscriber _sub_image;

    ros::Subscriber _sub_cmd_vel;
    ros::Subscriber _sub_tracked_object;
    ros::Publisher _pub_cmd_tld;
    ros::Subscriber _sub_center_view;
    ros::Subscriber _sub_followed_object;
    ros::Publisher _pub_controller_cmd;
    ros::ServiceClient _cl_controll_cmd;

    cv_bridge::CvImageConstPtr _cv_image;


    void image_receivedCB(const sensor_msgs::ImageConstPtr & msg);
    void tracked_objectCB(const tld_msgs::BoundingBoxConstPtr & msg);
    void imageCb(const sensor_msgs::ImageConstPtr & msg);
    void cmdCb(const geometry_msgs::Twist &msg);
    void tracked_object_cb(const tld_msgs::BoundingBox & msg);
    void center_view_cb(const geometry_msgs::Point &msg);
    void followed_object(const geometry_msgs::Point &msg);

    void send_center_image_request();

    int count_cmd;

public slots:
     void start_tracking(bool click);
     void btn_land_click();
     void btn_toglestate_click();

signals:
    void sig_image_received(const QImage & image);
    void sig_tracked_object_changed(const QRectF & bb);
    void sig_center_view_changed(const QPoint &p);
    void sig_drone_cmd_received(const geometry_msgs::Twist &cmd);
    void sig_followed_object_size(const int width,const int height);

protected:
    // keyboard control.... this is the only way i managed to do this...
    void keyPressEvent( QKeyEvent * key);
    void keyReleaseEvent( QKeyEvent * key);
};

#endif // MAINWINDOW_H
