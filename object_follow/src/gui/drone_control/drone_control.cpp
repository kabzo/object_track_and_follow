
#include "drone_control.h"

#include "std_msgs/Empty.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>

#include <sensor_msgs/image_encodings.h>
#include <tf/tf.h>

namespace enc = sensor_msgs::image_encodings;

pthread_mutex_t Drone_control::ardrone_CS = PTHREAD_MUTEX_INITIALIZER;


int Drone_control::getMS(ros::Time stamp ){
    if(ros_start_time == 0)
    {
        ros_start_time = stamp.sec;
        std::cout << "set ts base to " << ros_start_time << std::endl;
    }
    int mss = (stamp.sec - ros_start_time) * 1000 + stamp.nsec/1000000;

    if(mss < 0)
        std::cout << "ERROR: negative timestamp..."<< std::endl;
    return mss;
}

Drone_control::Drone_control(QWidget *parent) :
    QWidget(parent)//,it_(_n)
{
    setupUi(this);

    currentControlSource = CONTROL_KB;
    sensGaz = sensYaw = sensRP = 1;

    ros_start_time=0;


    connect(btn_w,SIGNAL(released()),this,SLOT(on_btn_released()));
    connect(btn_s,SIGNAL(released()),this,SLOT(on_btn_released()));
    connect(btn_a,SIGNAL(released()),this,SLOT(on_btn_released()));
    connect(btn_d,SIGNAL(released()),this,SLOT(on_btn_released()));

    QObject::connect(this,SIGNAL(sig_battery_changed(int)),progressBar_battery,SLOT(setValue(int)));

    for (int i = 0 ; i < 8 ; i ++){
        isPressed[i] = false;
    }

    _takeoff_pub	   = _n.advertise<std_msgs::Empty>(_n.resolveName("ardrone/takeoff"),1);
    _land_pub	   = _n.advertise<std_msgs::Empty>(_n.resolveName("ardrone/land"),1);
    _toggleState_pub	= _n.advertise<std_msgs::Empty>(_n.resolveName("ardrone/reset"),1);
    _pub_cmd_vel = _n.advertise<geometry_msgs::Twist>("cmd_vel",1);

    _sub_navdata = _n.subscribe("ardrone/navdata",1000,&Drone_control::navdataCB,this);

    _pub_toggleState = _n.advertise<std_msgs::Empty>(("ardrone/reset"),1);

    _cl_flat_trim = _n.serviceClient<std_srvs::Empty>(_n.resolveName("ardrone/flattrim"),1);

    _pub_apm_quaternion = _n.advertise<geometry_msgs::Quaternion>("setpoint_attitude",1);
    _pub_apm_thrust = _n.advertise<geometry_msgs::Twist>("set_cmd_vel",1);

    rdbtn_ardrone->setChecked(true);
    rdbtn_apm->setChecked(false);
}

Drone_control::~Drone_control()
{
}

void Drone_control::on_btn_land_clicked()
{
    pthread_mutex_lock(&ardrone_CS);
    _land_pub.publish(std_msgs::Empty());
    pthread_mutex_unlock(&ardrone_CS);

}

void Drone_control::on_btn_take_off_clicked()
{
    pthread_mutex_lock(&ardrone_CS);
    _takeoff_pub.publish(std_msgs::Empty());
    pthread_mutex_unlock(&ardrone_CS);

}

// KB control stuff
int Drone_control::mapKey(int k)
{
    switch(k)
    {
    case  Qt::Key_A: //j
        return 0;
    case Qt::Key_S: //k
        return 1;
    case Qt::Key_D: //l
        return 2;
    case Qt::Key_W: //i
        return 3;
    case Qt::Key_Q: //u
        return 4;
    case Qt::Key_E: //o
        return 5;
    case Qt::Key_R: //q
        return 6;
    case Qt::Key_F: //a
        return 7;
    }
    return -1;
}

void Drone_control::keyReleaseEvent( QKeyEvent * key)
{
    if(currentControlSource == CONTROL_KB)
    {
        int idx = mapKey(key->key());
        if(idx >= 0)
        {
            bool changed = false;
            if(!key->isAutoRepeat())	// ignore autorepeat-releases (!)
            {
                changed = isPressed[idx];
                isPressed[idx] = false;
            }

            if(changed)
                sendControlToDrone(calcKBControl());
        }
    }
}

void Drone_control::keyPressEvent( QKeyEvent * key)
{

    if(currentControlSource == CONTROL_KB)
    {
        int idx = mapKey(key->key());
        if(idx >= 0)
        {
            bool changed = !isPressed[idx];

            isPressed[idx] = true;
            lastRepeat[idx] = getMS();

            if(changed)
                sendControlToDrone(calcKBControl());
        }
    }


    if(key->key() == 16777216)	// ESC
    {
        setFocus();
        currentControlSource = (CONTROL_KB);
    }


    //    if(key->key() == 16777264)	// F1
    //    {
    //        rosThread->sendToggleState();
    //    }
}


ControlCommand Drone_control::calcKBControl()
{
    // clear keys that have not been refreshed for 1s, it is set to "not pressed"
    for(int i=0;i<8;i++)
        isPressed[i] = isPressed[i] && ((lastRepeat[i] + 1000) > getMS());

    ControlCommand c;

    if(isPressed[0]) c.roll = -sensRP; // j
    if(isPressed[1]) c.pitch = sensRP; // k
    if(isPressed[2]) c.roll = sensRP; // l
    if(isPressed[3]) c.pitch = -sensRP; // i
    if(isPressed[4]) c.yaw = -sensYaw; // u
    if(isPressed[5]) c.yaw = sensYaw; // o
    if(isPressed[6]) c.gaz = sensRP; // q
    if(isPressed[7]) c.gaz = -sensRP; // a

    return c;
}

void Drone_control::sendControlToArdrone(ControlCommand cmd){
    // TODO: check converstion (!)
    geometry_msgs::Twist cmdT;
    cmdT.angular.z = (double)-cmd.yaw;
    cmdT.linear.z = (double)cmd.gaz;
    cmdT.linear.x = (double)-cmd.pitch;
    cmdT.linear.y = (double)-cmd.roll;


    cmdT.angular.x = cmdT.angular.y = (double) 0;

    pthread_mutex_lock(&ardrone_CS);
    _pub_cmd_vel.publish(cmdT);
    pthread_mutex_unlock(&ardrone_CS);
}


void Drone_control::sendControlToDrone(ControlCommand cmd)
{

    if(rdbtn_apm->isChecked())
        sendControlToDroneAPM(cmd);
    else
        sendControlToArdrone(cmd);


    QString text = "Command:roll:";
    text.append(QString::number(cmd.roll));
    text.append(",pitch:");
    text.append(QString::number(cmd.pitch));
    text.append(",yaw:");
    text.append(QString::number(cmd.yaw));
    text.append(",trust:");
    text.append(QString::number(cmd.gaz));
    plainTextSendCommand->appendPlainText(text);
}

void Drone_control::sendControlToDroneAPM(ControlCommand cmd)
{
    geometry_msgs::Quaternion msg;
    tf::Quaternion q;
    q.setRPY(cmd.roll*M_PI/180,cmd.pitch*M_PI/180,cmd.yaw*M_PI/180);
    tf::quaternionTFToMsg(q,msg);
    _pub_apm_quaternion.publish(msg);

    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.z = cmd.gaz;

    pthread_mutex_lock(&ardrone_CS);
    _pub_apm_thrust.publish(cmd_vel);
    pthread_mutex_unlock(&ardrone_CS);
}

void Drone_control::navdataCB(const ardrone_autonomy::Navdata &msg){
    emit sig_battery_changed((int)msg.batteryPercent);

}

void Drone_control::on_btn_w_pressed()
{
    sendControlToDrone(ControlCommand(0,-sensRP,0,0));

}

void Drone_control::on_btn_released()
{
    sendControlToDrone(ControlCommand(0,0,0,0));

}

void Drone_control::on_btn_d_pressed()
{
    sendControlToDrone(ControlCommand(sensRP,0,0,0));

}

void Drone_control::on_btn_a_pressed()
{
    sendControlToDrone(ControlCommand(-sensRP,0,0,0));

}

void Drone_control::on_btn_s_pressed()
{
    sendControlToDrone(ControlCommand(0,sensRP,0,0));

}

void Drone_control::on_btn_trim_clicked()
{
    std_srvs::Empty trim;
    pthread_mutex_lock(&ardrone_CS);
    _cl_flat_trim.call(trim);
    pthread_mutex_unlock(&ardrone_CS);

}

void Drone_control::on_btn_toogleState_clicked()
{
    pthread_mutex_lock(&ardrone_CS);
    _pub_toggleState.publish(std_msgs::Empty());
    pthread_mutex_unlock(&ardrone_CS);

}

void Drone_control::on_rdbtn_apm_clicked(bool checked)
{
    if(checked){
        rdbtn_ardrone->setChecked(false);
        spin_response->setValue(10);

    }else{
        rdbtn_ardrone->setChecked(true);
        spin_response->setValue(1);

    }
}

void Drone_control::on_rdbtn_ardrone_clicked(bool checked)
{
    if(checked){
        rdbtn_apm->setChecked(false);
        spin_response->setValue(1);
    }else{
        spin_response->setValue(10);
        rdbtn_apm->setChecked(true);
    }
}

void Drone_control::on_spin_response_valueChanged(int arg1)
{
    sensGaz = sensRP = sensYaw = arg1;
}
