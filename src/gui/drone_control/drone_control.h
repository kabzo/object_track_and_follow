#ifndef DRONE_CONTROL_H
#define DRONE_CONTROL_H

#include "ui_drone_control.h"

#include <QWidget>
#include <QKeyEvent>

#include <ros/ros.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <tld_msgs/Target.h>
#include <tld_msgs/BoundingBox.h>
#include <std_msgs/Char.h>
#include <std_msgs/Float32.h>
#include "std_srvs/Empty.h"

#include <ardrone_autonomy/Navdata.h>




struct ControlCommand
{
    inline ControlCommand() {roll = pitch = yaw = gaz = 0;}
    inline ControlCommand(double roll, double pitch, double yaw, double gaz)
    {
        this->roll = roll;
        this->pitch = pitch;
        this->yaw = yaw;
        this->gaz = gaz;
    }

    inline double get_roll(){return roll;}
    double yaw, roll, pitch, gaz;
};


enum ControlSource {CONTROL_KB = 0, CONTROL_JOY = 1, CONTROL_AUTO = 2, CONTROL_NONE = 3};


class Drone_control : public QWidget, public Ui::Drone_control
{
    Q_OBJECT
private:
    ros::NodeHandle _n;

    ros::Publisher _takeoff_pub;
    ros::Publisher _land_pub;
    ros::Publisher _toggleState_pub;
    ros::Publisher _pub_cmd_vel;
    ros::Publisher _pub_toggleState;

    ros::Publisher _pub_apm_quaternion;
    ros::Publisher _pub_apm_thrust;

    ros::Subscriber _sub_navdata;

    ros::ServiceClient _cl_flat_trim;

    double sensGaz, sensYaw, sensRP;
    ControlSource currentControlSource;


    cv_bridge::CvImageConstPtr cv_ptr;
    bool first_image;

    static pthread_mutex_t ardrone_CS;

public:
    explicit Drone_control(QWidget *parent = 0);
    ~Drone_control();
    void sendControlToDrone(ControlCommand cmd);
    void sendControlToArdrone(ControlCommand cmd);
    void sendControlToDroneAPM(ControlCommand cmd);

    ControlCommand calcKBControl();

    unsigned int ros_start_time;
    int getMS(ros::Time stamp = ros::Time::now());

    void image_receivedCB(const sensor_msgs::ImageConstPtr & msg);
    void navdataCB(const ardrone_autonomy::Navdata &msg);

    void set_nodeHandle(ros::NodeHandle &node);

    void keyPressEvent( QKeyEvent * key);
    void keyReleaseEvent( QKeyEvent * key);

public slots:
    void on_btn_land_clicked();
    void on_btn_take_off_clicked();

    void on_btn_w_pressed();
    void on_btn_released();

    void on_btn_d_pressed();

    void on_btn_a_pressed();

    void on_btn_s_pressed();

    void on_btn_trim_clicked();

    void on_btn_toogleState_clicked();

protected:
    // keyboard control.... this is the only way i managed to do this...

    int mapKey(int k);
    bool isPressed[8];	//{j k l i u o q a}
    unsigned int lastRepeat[8];

signals:
    void sig_image_received(const QImage & image);
    void sig_battery_changed(int val);

private slots:
    void on_rdbtn_apm_clicked(bool checked);
    void on_rdbtn_ardrone_clicked(bool checked);
    void on_spin_response_valueChanged(int arg1);
};

#endif // DRONE_CONTROL_H
