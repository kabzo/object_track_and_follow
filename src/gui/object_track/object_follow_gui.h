#ifndef OBJECT_FOLLOW_GUI_H
#define OBJECT_FOLLOW_GUI_H

#include <QWidget>

#include <QtGui/QWidget>
#include <QtGui/QKeyEvent>
#include <QtCore/QObject>
#include <QtGui/QImage>
#include <QtCore/QRectF>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <tld_msgs/Target.h>
#include <tld_msgs/BoundingBox.h>
#include <std_msgs/Char.h>
#include <std_msgs/Float32.h>

#include "ui_object_follow_gui.h"
#include <image_transport/image_transport.h>



class object_follow_gui : public QWidget, private Ui::object_follow_gui
{
    Q_OBJECT

public:
    explicit object_follow_gui(QWidget *parent = 0);
//    ~object_follow_gui();

    void set_nodeHandle(ros::NodeHandle &node);

    ros::NodeHandle n;
    image_transport::ImageTransport it_;
    ros::Publisher pub1;
    ros::Publisher pub2;
    image_transport::Subscriber sub1;
    ros::Subscriber sub2;
    ros::Subscriber sub3;

protected:
    void keyPressEvent(QKeyEvent * event);

private:
//    Ui::object_follow_gui *ui;
    cv_bridge::CvImageConstPtr cv_ptr;
    bool first_image;

    void image_receivedCB(const sensor_msgs::ImageConstPtr & msg);
    void tracked_objectCB(const tld_msgs::BoundingBoxConstPtr & msg);
    void fps_trackerCB(const  std_msgs::Float32 & msg);

signals:
    void sig_image_received(const QImage & image);
    void sig_tracked_object_changed(const QRectF & bb);
    void sig_fps_tracker_changed(int fps);
    void sig_confidence_changed(int confidence);


public slots:
    void clear_background();
    void clear_and_stop_tracking();
    void toggle_learning();
    void alternating_mode();
    void export_model();
    void import_model();
    void reset();
private slots:

};

#endif // OBJECT_FOLLOW_GUI_H
