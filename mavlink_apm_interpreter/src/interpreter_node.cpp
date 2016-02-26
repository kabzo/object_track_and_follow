
#include <ros/ros.h>
#include <geometry_msgs/Quaternion.h>

#include <mavros/RCIn.h>
#include <mavros/RCOut.h>
#include <mavros/OverrideRCIn.h>
#include <mavros/ParamGet.h>
#include <mavros/State.h>

#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp>

#include <tf/tf.h>

#include "RC_Channel.h"

RC_Channel rc1,rc2,rc3,rc4;

#define MAX_ANGLE 4500

#define constrain(x,min,max) x<min?min:(x>max?max:x)

float throttle_deadzone = 0;
long throttle_min, throttle_max;
long pilot_velocity_z_max;


ros::Subscriber sub_set_attitude;
ros::Subscriber sub_set_throttle;
ros::Publisher pub_rc_override;
ros::Subscriber sub_state;

mavros::State vehicle_state;

void override_rc(long roll_pwm, long pitch_pwm, long yaw_pwm, long trust_pwm){
  mavros::OverrideRCIn override;

  if(vehicle_state.mode == "ALT_HOLD"){
      override.channels[0] = roll_pwm;
      override.channels[1] = pitch_pwm;
      override.channels[2] = trust_pwm; //m/s to cm/s
      override.channels[3] = yaw_pwm;
      override.channels[4] = 65535;
      override.channels[5] = 65535;
      override.channels[6] = 65535;
      override.channels[7] = 65535;
    }else{
      override.channels.assign(0);
    }
  pub_rc_override.publish(override);


}

float get_trust_pwm(float desired_rate){
  // throttle failsafe check

  float throttle_control = 0.0f;
  float mid_stick = rc3.get_control_mid();
  float deadband_top = mid_stick + throttle_deadzone;
  float deadband_bottom = mid_stick - throttle_deadzone;


  // ensure a reasonable deadzone
  throttle_deadzone = constrain(throttle_deadzone, 0, 400);

  // check throttle is above, below or in the deadband
  if (desired_rate < 0) {
      // below the deadband
      throttle_control = (desired_rate/(float)pilot_velocity_z_max)*(deadband_bottom-throttle_min)+deadband_bottom;

    }else if (desired_rate > 0) {
      // above the deadband
      throttle_control = desired_rate/(float)pilot_velocity_z_max*(1000.0f-deadband_top)+deadband_top;
    }else{
      // must be in the deadband
      throttle_control = mid_stick;
    }

  // ensure a reasonable throttle value
  throttle_control = constrain(throttle_control,throttle_min,1000.0f);


  // desired climb rate for logging
ROS_INFO("thcon:%g",throttle_control);
  return throttle_control;
}

void attitude_cb(const geometry_msgs::Quaternion &msg){
  tf::Quaternion q;
  tf::quaternionMsgToTF(msg,q);
  double roll,pitch,yaw;
  tf::Matrix3x3(q).getRPY(roll,pitch,yaw);
  roll*=180/M_PI*100;  //in centidegrees
  pitch*=180/M_PI*100;
  yaw*=180/M_PI*100;


  //    mavros::OverrideRCIn msg_override;
  long rc1_pwm = rc1.calc_pwm(roll);
  long rc2_pwm = rc2.calc_pwm(pitch);
  long rc4_pwm = rc4.calc_pwm(yaw);

  //    ROS_INFO("r:%g,pwm:%ld ; p:%g,pwm:%ld ; y:%g,pwm:%ld",roll,rc1_pwm,pitch,rc2_pwm,yaw,rc4_pwm);

  override_rc(rc1_pwm,rc2_pwm,rc4_pwm,rc3.output);


}

void cmd_cb(const geometry_msgs::Twist &msg){

  long rc3_pwm = rc3.calc_pwm(get_trust_pwm(msg.linear.z*100));

//      ROS_INFO("throttle:%ld, lin.z:%d",rc3_pwm,rc3.percent_input());

  override_rc(rc1.output,rc2.output,rc4.output,rc3_pwm);


}



double requst_param(ros::ServiceClient &cl,std::string name){
  mavros::ParamGet param;
  param.request.param_id =name;
  int i;
  for(i = 0; i<5 && param.response.success == false;i++){
      cl.call(param);
    }
  if(i==5){
      ROS_INFO("ERROR IN CALL");
      return -111.1;
    }else
    return param.response.integer==0?param.response.real:param.response.integer;
}

void init_params(ros::ServiceClient cl_param_get){

  RC_Channel::Property rc1_p;
  rc1_p.max = requst_param(cl_param_get,"RC1_MAX");
  rc1_p.min = requst_param(cl_param_get,"RC1_MIN");
  rc1_p.trim = requst_param(cl_param_get,"RC1_TRIM");
  rc1.set_property(rc1_p);

  RC_Channel::Property rc2_p;
  rc2_p.max = requst_param(cl_param_get,"RC2_MAX");
  rc2_p.min = requst_param(cl_param_get,"RC2_MIN");
  rc2_p.trim = requst_param(cl_param_get,"RC2_TRIM");
  rc2.set_property(rc2_p);

  RC_Channel::Property rc3_p;
  rc3_p.max = requst_param(cl_param_get,"RC3_MAX");
  rc3_p.min = requst_param(cl_param_get,"RC3_MIN");
  rc3_p.trim = requst_param(cl_param_get,"RC3_TRIM");
  rc3.set_property(rc3_p);

  RC_Channel::Property rc4_p;
  rc4_p.max = requst_param(cl_param_get,"RC4_MAX");
  rc4_p.min = requst_param(cl_param_get,"RC4_MIN");
  rc4_p.trim = requst_param(cl_param_get,"RC4_TRIM");
  rc4.set_property(rc4_p);


  ROS_INFO("RC1, max:%ld, min:%ld, trim:%ld",rc1.get_property().max,rc1.get_property().min,rc1.get_property().trim);
  ROS_INFO("RC2, max:%ld, min:%ld, trim:%ld",rc2.get_property().max,rc2.get_property().min,rc2.get_property().trim);
  ROS_INFO("RC3, max:%ld, min:%ld, trim:%ld",rc3.get_property().max,rc3.get_property().min,rc3.get_property().trim);
  ROS_INFO("RC4, max:%ld, min:%ld, trim:%ld",rc4.get_property().max,rc4.get_property().min,rc4.get_property().trim);


  rc1.set_angle(MAX_ANGLE);
  rc2.set_angle(MAX_ANGLE);
  rc4.set_angle(MAX_ANGLE);

  pilot_velocity_z_max =  requst_param(cl_param_get,"PILOT_VELZ_MAX");
  throttle_max = requst_param(cl_param_get,"THR_MAX");
  throttle_min = requst_param(cl_param_get,"THR_MIN");
  rc3.set_range(throttle_min,throttle_max);


  ROS_INFO(" THR_MAX:%ld, THR_MIN:%ld, PILOT_VELZ_MAX:%ld",throttle_max,throttle_min,pilot_velocity_z_max);

  rc1.calc_pwm(0);
  rc2.calc_pwm(0);
  rc3.calc_pwm(get_trust_pwm(0));
  rc4.calc_pwm(0);


}

void state_cb(const mavros::State &msg){
  if(vehicle_state.mode!=msg.mode){
      std::string m =  msg.mode;
      ROS_INFO("MODE:%s",m.c_str());
      vehicle_state = msg;
      override_rc(rc1.output,rc2.output,rc4.output,rc3.output);

    }



}

int main(int argc, char **argv)
{


  ros::init(argc, argv, "interpreter_node");
  ROS_INFO("Started interpreter_node Node.");

  ros::NodeHandle n;



  pub_rc_override = n.advertise<mavros::OverrideRCIn>("mavros/rc/override",10);
  sub_set_attitude = n.subscribe("setpoint_attitude",10,&attitude_cb);
  sub_set_throttle = n.subscribe("set_cmd_vel",10,&cmd_cb);
  sub_state = n.subscribe("mavros/state",10,&state_cb);

  ros::ServiceClient cl_param_get;

  cl_param_get = n.serviceClient<mavros::ParamGet>("mavros/param/get",1);

  init_params(cl_param_get);

  ros::spin();

}

