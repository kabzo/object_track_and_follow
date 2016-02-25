
#include <ros/ros.h>

#include <boost/interprocess/sync/interprocess_mutex.hpp>

#include <cv_bridge/cv_bridge.h>

#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <std_msgs/Header.h>
#include <sensor_msgs/Image.h>
#include <tld_msgs/Target.h>
#include <tld_msgs/BoundingBox.h>
#include <std_msgs/Char.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <ardrone_autonomy/Navdata.h>


#include <string>
#include <math.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/UInt8MultiArray.h>

#include "Controller.h"

#include <dynamic_reconfigure/server.h>
#include <object_follow/pidParamConfig.h>
#include <object_follow/controller_cmd.h>

#include <camera_info_manager/camera_info_manager.h>



#define CAMERA_MATRIX  {564.445569348335, 0, 324.238794500839, 0, 562.403768220257, 194.367064448638, 0, 0, 1}

#define PROJECTION_MATRIX {456.138549804688, 0, 316.764636943721, 0, 0, 524.119567871094, 192.59496666532, 0, 0, 0, 1, 0}
#define FRONTCAM_RESOLUTION_WIDTH           640.0
#define FRONTCAM_RESOLUTION_HEIGHT          360
#define MULTIROTOR_FRONTCAM_HORIZONTAL_ANGLE_OF_VIEW   70
#define MULTIROTOR_FRONTCAM_VERTICAL_ANGLE_OF_VIEW     38
#define MULTIROTOR_FRONTCAM_ALPHAX                     460
#define MULTIROTOR_FRONTCAM_ALPHAY                     530
#define MULTIROTOR_IBVSCONTROLLER_INIT_DEPTH           3
#define MULTIROTOR_IBVSCONTROLLER_TARGET_INIT_SIZE     0.12

//#define MULTIROTOR_FRONTCAM_C_fx2Dy  ( MULTIROTOR_IBVSCONTROLLER_INIT_DEPTH*MULTIROTOR_FRONTCAM_RESOLUTION_WIDTH/(MULTIROTOR_FRONTCAM_ALPHAX) )
//#define MULTIROTOR_FRONTCAM_C_fx2DY  ( MULTIROTOR_FRONTCAM_HORIZONTAL_ANGLE_OF_VIEW * (M_PI/180.0) )
//#define MULTIROTOR_FRONTCAM_C_fy2Dz  ( MULTIROTOR_IBVSCONTROLLER_INIT_DEPTH*MULTIROTOR_FRONTCAM_RESOLUTION_HEIGHT/(MULTIROTOR_FRONTCAM_ALPHAY) )
//#define MULTIROTOR_FRONTCAM_C_fD2Dx  ( sqrt( (MULTIROTOR_FRONTCAM_ALPHAX*MULTIROTOR_FRONTCAM_ALPHAY*MULTIROTOR_IBVSCONTROLLER_TARGET_INIT_SIZE)/(MULTIROTOR_FRONTCAM_RESOLUTION_WIDTH*MULTIROTOR_FRONTCAM_RESOLUTION_HEIGHT) ) )
//#define MULTIROTOR_FRONTCAM_C_DY2Dfx ( 1/(MULTIROTOR_FRONTCAM_C_fx2DY) )
//#define MULTIROTOR_FRONTCAM_C_DP2Dfy ( 1/( MULTIROTOR_FRONTCAM_VERTICAL_ANGLE_OF_VIEW * (M_PI/180.0) ) )

#define MULTIROTOR_IBVSCONTROLLER_INITVAL_FX  0.5
#define MULTIROTOR_IBVSCONTROLLER_INITVAL_FY  0.5
#define MULTIROTOR_IBVSCONTROLLER_INITVAL_FS  0.025
//#define MULTIROTOR_IBVSCONTROLLER_INITVAL_FD  ( 1.0/sqrt(MULTIROTOR_IBVSCONTROLLER_INITVAL_FS) )


//#define C_fx2Dy   MULTIROTOR_FRONTCAM_C_fx2Dy
//#define C_fx2DY   MULTIROTOR_FRONTCAM_C_fx2DY
//#define C_fy2Dz   MULTIROTOR_FRONTCAM_C_fy2Dz
//#define C_fD2Dx   MULTIROTOR_FRONTCAM_C_fD2Dx
//#define C_DY2Dfx  MULTIROTOR_FRONTCAM_C_DY2Dfx
//#define C_DP2Dfy  MULTIROTOR_FRONTCAM_C_DP2Dfy


enum Flight_Modes{
    HOVER,
    TRACKING
};

class TrackedRect:public cv::Rect
{
public:
TrackedRect():cv::Rect(){
}

    TrackedRect(int xc,int yc, int w,int h):cv::Rect(xc,yc,w,h){
        seted = false;
        set_point.x = x+width/2;
        set_point.y = y+height/2;
        set_point.z = (w*h);

    }

    TrackedRect(int xc,int yc, int w,int h, double yaw):cv::Rect(xc,yc,w,h){
        seted = false;
        set_point.x = x+width/2;
        set_point.y = y+height/2;
        set_point.z = (w*h);
        set_point.yaw = yaw;

    }

    void set(int xc,int yc, int w,int h, double yaw){
        x= xc;
        y= yc;
        width = w;
        height = h;

        set_point.x = ( (float) xc + ((float) w)/2.0 )/FRONTCAM_RESOLUTION_WIDTH;
        set_point.y = ( (float) yc + ((float)h)/2.0 )/FRONTCAM_RESOLUTION_HEIGHT;

        // [fs]size and [fD]inverse sqrt of size of object in the image
        set_point.z = 1.0/sqrt( (((float)w)/FRONTCAM_RESOLUTION_WIDTH)*(((float)h)/FRONTCAM_RESOLUTION_HEIGHT));
        set_point.yaw = (float)yaw ;
    }

    void apply_pitch_degrees(float pitch,float foxy){
      set_point.y *= FRONTCAM_RESOLUTION_HEIGHT;
      set_point.y -=pitch*(FRONTCAM_RESOLUTION_HEIGHT/foxy);
      set_point.y /= FRONTCAM_RESOLUTION_HEIGHT;
    }

    Point4d set_point;
    bool seted;
};

class FollowControll{

private:
    ros::NodeHandle _n;

    ros::Subscriber _sub_tracked_object;
    ros::Subscriber _sub_tracked_fps;
    ros::Subscriber _sub_tracked_box;
    ros::Subscriber _sub_center_view;
    ros::Subscriber _sub_nav_data;

    ros::Publisher _pub_cmd_vel;
    ros::Publisher _pub_cmd_tld;
    ros::Publisher _pub_followed_object_width_height;
    ros::Publisher _pub_tracking_error;

    ros::ServiceServer _srv_controll_cmd;

    Controller *_pid;
    Point4d _error_pid;
    TrackedRect _tracker_rect_reference;
    double _min_fly_confidance;

    TrackedRect _tracked_rect;
    bool _found_tracked_object;
    bool set_reference_target_width_height;

     double fovx,fovy,focalLength,aspectRatio;

    int _tracke_fps;
    Flight_Modes flight_mode;
    ardrone_autonomy::Navdata navdata_telemetry;

    dynamic_reconfigure::Server<object_follow::pidParamConfig> srv;
    dynamic_reconfigure::Server<object_follow::pidParamConfig>::CallbackType f;

    int controller_rate;

    void center_view_cb(const geometry_msgs::Point &p);
public:

    FollowControll();

    void dyn_recon_callback(object_follow::pidParamConfig &config, uint32_t level);

    void follow();
    void publish_command();

    void tracked_object_cb(const tld_msgs::BoundingBox & msg);
    void tracked_fps_cb(const std_msgs::Float32 &msg);
    void image_cb(const sensor_msgs::ImageConstPtr & msg);
    void controller_cmd_cb(const std_msgs::UInt8MultiArray &msg);
    void navdata_cb(const ardrone_autonomy::Navdata &msg);

    bool controller_cmd(object_follow::controller_cmd::Request &reg,
                        object_follow::controller_cmd::Response &res);

};
