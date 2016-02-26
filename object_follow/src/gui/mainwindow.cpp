#include "mainwindow.h"

#include <sensor_msgs/image_encodings.h>
#include <QMetaType>
#include <std_msgs/UInt8MultiArray.h>
#include <opencv2/highgui/highgui.hpp>

Q_DECLARE_METATYPE(geometry_msgs::Twist)

pthread_mutex_t MainWindow::send_CS = PTHREAD_MUTEX_INITIALIZER;


MainWindow::~MainWindow()
{
}

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),it_(_n)

{
    setupUi(this);

    qRegisterMetaType<geometry_msgs::Twist>("geometry_msgs::Twist");

    QObject::connect(this,SIGNAL(sig_image_received(const QImage &)),drone_contro_graphics_view,SLOT(image_received(const QImage &)));
    QObject::connect(this,SIGNAL(sig_tracked_object_changed(const QRectF &)),drone_contro_graphics_view,SLOT(tracked_objet_changed(const QRectF &)));
    QObject::connect(this,SIGNAL(sig_center_view_changed(const QPoint &)),drone_contro_graphics_view,SLOT(center_view_changed(const QPoint &)));
    QObject::connect(this,SIGNAL(sig_drone_cmd_received(const geometry_msgs::Twist &)),drone_contro_graphics_view,SLOT(cmd_received(const geometry_msgs::Twist &)));
    QObject::connect(this,SIGNAL(sig_followed_object_size(int,int)),drone_contro_graphics_view,SLOT(followed_object_size(int,int)));
    QObject::connect(checkBox_startTracking,SIGNAL(clicked(bool)),this,SLOT(start_tracking(bool)));
    QObject::connect(btn_land,SIGNAL(clicked(bool)),this,SLOT(btn_land_click()));
    QObject::connect(btn_toglestate,SIGNAL(clicked(bool)),this,SLOT(btn_toglestate_click()));

    ros::NodeHandle np("~");
    std::string image_sub;
    np.param("image", image_sub, std::string("image"));


    count_cmd = 0;

    _sub_image =  it_.subscribe(image_sub, 1000, &MainWindow::imageCb, this);
    _sub_cmd_vel = _n.subscribe("cmd_vel",1000,&MainWindow::cmdCb,this);
    _sub_tracked_object = _n.subscribe("tld_tracked_object", 1000, &MainWindow::tracked_object_cb, this);
    _pub_cmd_tld = _n.advertise<std_msgs::Char>("tld_gui_cmds", 1000, true);
    _sub_center_view = _n.subscribe("tld_center_camera",1000,&MainWindow::center_view_cb,this);
    _pub_controller_cmd = _n.advertise<std_msgs::UInt8MultiArray>("controller_cmd",1,true);
    _sub_followed_object = _n.subscribe("followed_object",1000,&MainWindow::followed_object,this);

    _cl_controll_cmd = _n.serviceClient<object_follow::controller_cmd>("controller_cmd");

    std::cout<<_sub_image.getTopic()<<std::endl;


}

void MainWindow::imageCb(const sensor_msgs::ImageConstPtr & msg)
{

    //    mutex.lock();
    if (sensor_msgs::image_encodings::isColor(msg->encoding))
        _cv_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
    else
    {
        _cv_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
        cv::cvtColor(_cv_image->image, _cv_image->image, CV_GRAY2BGR);
    }

    QImage image;

    if (sensor_msgs::image_encodings::isColor(msg->encoding))
    {
        image = QImage((const unsigned char*)(_cv_image->image.data),_cv_image->image.cols,_cv_image->image.rows,1920,QImage::Format_RGB888);
        //image.rgbSwapped();
    }
    else
    {
        image = QImage((const unsigned char*)(_cv_image->image.data),_cv_image->image.cols,_cv_image->image.rows,1920,QImage::Format_Indexed8);
    }

    if(drone_contro_graphics_view->get_target_drone_view()->center.x() == 0 && drone_contro_graphics_view->get_target_drone_view()->center.y() == 0 )
    {
        std_msgs::Char cmd;
        cmd.data = 'v';
        _pub_cmd_tld.publish(cmd);
    }
    emit sig_image_received(image);
}
void MainWindow::center_view_cb(const geometry_msgs::Point &msg)
{
    QPoint p (msg.x,msg.y);
    emit sig_center_view_changed(p);
}

void MainWindow::followed_object(const geometry_msgs::Point &msg){
    emit sig_followed_object_size(msg.x,msg.y);
}



void MainWindow::cmdCb(const geometry_msgs::Twist &msg)
{
    if(count_cmd++==5)
    {
        QString text = "l.x:";
        text.append(QString::number(msg.linear.x));
        text.append(",l.y:");
        text.append(QString::number(msg.linear.y));
        text.append(",l.z:");
        text.append(QString::number(msg.linear.z));
        text.append(",a.z:");
        text.append(QString::number(msg.angular.z));
        textEdit->append(text);
        count_cmd =0;

        QTextCursor c = textEdit->textCursor();
        c.movePosition(QTextCursor::End);
        textEdit->setTextCursor(c);

    }
    emit sig_drone_cmd_received(msg);
}

void MainWindow::btn_land_click(){
    drone_control->on_btn_land_clicked();
}

void MainWindow::btn_toglestate_click(){
    drone_control->on_btn_toogleState_clicked();
}

void MainWindow::send_center_image_request()
{
    std_msgs::Char cmd;
    cmd.data = 'v';
    pthread_mutex_lock(&send_CS);
    _pub_cmd_tld.publish(cmd);
    pthread_mutex_unlock(&send_CS);
}


void MainWindow::start_tracking(bool click)
{
    object_follow::controller_cmd cmd;

    cmd.request.str = "follow_and_track";
    cmd.request.mode = (int)click;

    pthread_mutex_lock(&send_CS);
    bool b = _cl_controll_cmd.call(cmd);
    pthread_mutex_unlock(&send_CS);

    if(b)
    {
        if(cmd.response.res)
            checkBox_startTracking->setChecked(click);
        else
        {
            checkBox_startTracking->setChecked(!click);

        }
    }
    else
    {
        checkBox_startTracking->setChecked(false);
    }
}

void MainWindow::tracked_object_cb(const tld_msgs::BoundingBox & msg)
{
    QRectF rect(msg.x,msg.y,msg.width,msg.height);
    emit sig_tracked_object_changed(rect);
}



void MainWindow::keyReleaseEvent( QKeyEvent * key)
{
    drone_control->keyReleaseEvent(key);
}

void MainWindow::keyPressEvent( QKeyEvent * key)
{
    drone_control->keyPressEvent(key);
}



