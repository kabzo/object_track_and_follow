#include "FollowControll.h"

FollowControll::FollowControll():
  set_reference_target_width_height(false),
  flight_mode(HOVER),
  controller_rate(100)

{
  _sub_tracked_object = _n.subscribe("tld_tracked_object", 1000, &FollowControll::tracked_object_cb, this);
  _sub_tracked_fps = _n.subscribe("tld_fps",1000,&FollowControll::tracked_fps_cb,this);
  _sub_center_view = _n.subscribe("tld_center_camera",1000,&FollowControll::center_view_cb,this);
  _sub_nav_data = _n.subscribe("ardrone/navdata",1000,&FollowControll::navdata_cb,this);

  _pub_cmd_vel = _n.advertise<geometry_msgs::Twist>("cmd_vel",1);
  _pub_cmd_tld = _n.advertise<std_msgs::Char>("tld_gui_cmds", 1000, true);
  _pub_followed_object_width_height = _n.advertise<geometry_msgs::Point>("followed_object",1);
  _pub_tracking_error = _n.advertise<geometry_msgs::Point>("tracking_error",1);

  _srv_controll_cmd = _n.advertiseService("controller_cmd",&FollowControll::controller_cmd,this);

  _pid = new Controller();

  std_msgs::Char cmd;
  cmd.data = 'v';
  _pub_cmd_tld.publish(cmd);

  f = boost::bind(&FollowControll::dyn_recon_callback,this, _1, _2);
  srv.setCallback(f);

  float cam_mat[9]= CAMERA_MATRIX;
  cv::Mat camera_matrix = cv::Mat(3, 3, CV_32F, cam_mat);
  cv::Point2d principle;
  cv::calibrationMatrixValues(camera_matrix,cv::Size(FRONTCAM_RESOLUTION_HEIGHT,FRONTCAM_RESOLUTION_WIDTH),
                              FRONTCAM_RESOLUTION_WIDTH,FRONTCAM_RESOLUTION_HEIGHT,
                              fovx,fovy,focalLength,principle,aspectRatio);



  float m[12]= PROJECTION_MATRIX;
  cv::Mat projection_matrix = cv::Mat(3, 4, CV_32FC1, m);
  std::cout<<projection_matrix<<std::endl;
  cv::Vec4f v(0/2,0/2,1,1);
  cv::Mat vec =camera_matrix * projection_matrix*cv::Mat(v);
  std::cout<<vec<<std::endl;

  ROS_INFO("fovx:%g,fovy:%g",fovx,fovy);

  _tracker_rect_reference.set_point.x = _tracker_rect_reference.set_point.y =
        _tracker_rect_reference.set_point.yaw = 1/2.0;
}


void FollowControll::tracked_object_cb(const tld_msgs::BoundingBox & msg)
{
  if( (msg.width> 1 || msg.height > 1) && (msg.confidence*100)>_min_fly_confidance)
    {
      _found_tracked_object = true;


      _tracked_rect.set(msg.x,msg.y,msg.width,msg.height,0);
      _tracked_rect.apply_pitch_degrees(navdata_telemetry.rotY,fovy);

      if(set_reference_target_width_height){
          set_reference_target_width_height = false;
          _tracker_rect_reference.set(FRONTCAM_RESOLUTION_WIDTH/2-msg.width/2,FRONTCAM_RESOLUTION_HEIGHT/2-msg.height/2,msg.width,msg.height,1/2.0);

          geometry_msgs::Point p;
          p.x = msg.width;
          p.y = msg.height;
          _pub_followed_object_width_height.publish(p);
        }

      float max_x_error = 2.0/fovx;
      float x_error = _tracker_rect_reference.set_point.x-_tracked_rect.set_point.x;
      if(fabs(x_error)>max_x_error){
          _tracked_rect.set_point.yaw =(x_error>0)?_tracked_rect.set_point.x+max_x_error:_tracked_rect.set_point.x-max_x_error;
        }
      else
         _tracked_rect.set_point.yaw =0.5;

      geometry_msgs::Point tracking_error;
      tracking_error.x = _tracked_rect.set_point.x - _tracker_rect_reference.set_point.x;
      tracking_error.y = _tracked_rect.set_point.y - _tracker_rect_reference.set_point.y;
      tracking_error.z = _tracked_rect.set_point.z - _tracker_rect_reference.set_point.z;
      _pub_tracking_error.publish(tracking_error);

    }
  else
    {
      _found_tracked_object = false;
    }

}

void FollowControll::center_view_cb(const geometry_msgs::Point &p)

{
  //  _camera_view_reference.x = p.x/FRONTCAM_RESOLUTION_WIDTH;
  //  _camera_view_reference.y = p.y/FRONTCAM_RESOLUTION_HEIGHT;
}

bool FollowControll::controller_cmd(object_follow::controller_cmd::Request &reg,
                                    object_follow::controller_cmd::Response &res)
{
  if (reg.str=="follow_and_track")
    {
      if(reg.mode == 1){
          set_reference_target_width_height = true;
          flight_mode = TRACKING;
        }
      else{
          flight_mode = HOVER;
          _error_pid = 0;
          _pid->reset_I();
          publish_command();
        }
      res.res = true;
    }

  return true;
}


void FollowControll::tracked_fps_cb(const std_msgs::Float32 &msg)
{
  _tracke_fps = msg.data;
}

void FollowControll::navdata_cb(const ardrone_autonomy::Navdata &msg){
  navdata_telemetry = msg;
}

void FollowControll::follow()
{
  ros::Rate loop_rate(controller_rate);
  double time_ms = ((double)(1.0/((double)controller_rate)))*1000.0;

  while (ros::ok())
    {

      if((!_tracker_rect_reference.set_point.is_zero() && flight_mode == TRACKING))
        {

          if(_found_tracked_object)
            {
              _error_pid = _pid->get_pid(_tracker_rect_reference.set_point,_tracked_rect.set_point,time_ms);

            }else{
              _error_pid = 0;
            }
          publish_command();

        }
      else
        {
          ROS_INFO("TRACKING IS OFF, FLIGHT MODE: %d",flight_mode);
          cv::waitKey(1000);
          //          if(_camera_view_reference.x<2 && _camera_view_reference.y<2){
          //              std_msgs::Char cmd;
          //              cmd.data = 'v';
          //              _pub_cmd_tld.publish(cmd);
          //            }
          geometry_msgs::Point p;
          p.x = 0;
          p.y = 0;
          _pub_followed_object_width_height.publish(p);


        }


      ros::spinOnce();
      loop_rate.sleep();
    }
}

void FollowControll::publish_command()
{
  geometry_msgs::Twist cmd;

  cmd.linear.x = -_error_pid.z;
  cmd.linear.z = _error_pid.y;

  cmd.angular.z = _error_pid.x;//cmd.linear.y>0?cmd.linear.y/10:-cmd.linear.y/10;


  cmd.angular.x = cmd.angular.y = (flight_mode == TRACKING)?1:0;
  _pub_cmd_vel.publish(cmd);
}

void FollowControll::dyn_recon_callback(object_follow::pidParamConfig &config, uint32_t level)
{
  ROS_INFO("Reconfigure request XY: %f, %f, %f",
           config.groups.pid_x.kPx,config.groups.pid_x.kDx,config.groups.pid_x.pMaxX);
  ROS_INFO("Reconfigure request Range: %f, %f, %f",
           config.groups.pid_range.kPrange,config.groups.pid_range.kDrange,config.groups.pid_range.pMaxRange);
  ROS_INFO("Reconfigure request Yaw: %f, %f, %f",
           config.groups.pid_yaw.kPyaw,config.groups.pid_yaw.kDyaw,config.groups.pid_yaw.pMaxYaw);
  ROS_INFO("LPF: %f",
           config.LPF);

  Const_PID x{config.groups.pid_x.kPx,0,config.groups.pid_x.kDx,config.groups.pid_x.pMaxX,0};
  Const_PID y{config.groups.pid_y.kPy,0,config.groups.pid_y.kDy,config.groups.pid_y.pMaxY,0};
  Const_PID range{ config.groups.pid_range.kPrange,0,config.groups.pid_range.kDrange,config.groups.pid_range.pMaxRange,0};
  Const_PID yaw{config.groups.pid_yaw.kPyaw,0,config.groups.pid_yaw.kDyaw,config.groups.pid_yaw.pMaxYaw,0};

  _pid->set_k_pid(x,y,range,yaw,_pid->set_d_lpf_alpha(config.LPF,(double)1.0/controller_rate));

  _min_fly_confidance = config.min_confidance;
}


