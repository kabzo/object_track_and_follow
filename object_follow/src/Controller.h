#include "math.h"
#include <opencv2/core/core.hpp>

#define PID_D_TERM_FILTER 0.556864f    // Default 100Hz Filter Rate with 20Hz Cutoff Frequency
//Constants used in some of the functions below
#define P_MAX 4500

#define ENABLED	true
#define DISABLED	false

#define constrain(amt,low,high) if(amt.x<low)amt.x=low;if(amt.y<low)amt.y=low;if(amt.z<low)amt.z=low;if(amt.x>high)amt.x=high;if(amt.y>high)amt.y=high;if(amt.z>high)amt.z=high;
#define constrainf(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

struct Const_PID{
    double cP,cI,cD;
    double pLim,iLim;
};

struct Point4d{
    Point4d(double xc,double yc,double zc, double yawc):x(xc),y(yc),z(zc),yaw(yawc){}
    Point4d(double a):x(a),y(a),z(a),yaw(a){}
    Point4d():x(0),y(0),z(0),yaw(0){}

    Point4d limit(double xLim,double yLim,double zLim, double yawLim){
        double xc = constrainf(x,-xLim,xLim);
        double yc = constrainf(y,-yLim,yLim);
        double zc = constrainf(z,-zLim,zLim);
        double yawc = constrainf(yaw,-yawLim,yawLim);
        return Point4d(xc,yc,zc,yawc);
    }

    bool is_zero(){
        return x == 0 && y == 0 && z == 0 && yaw == 0;
    }

    Point4d operator*(double k){
        return Point4d(x*k,y*k,z*k,yaw*k);
    }

    Point4d operator*(cv::Point3d  k){
        return Point4d(x*k.x,y*k.x,z*k.y,yaw*k.z);
    }

   Point4d& operator=(double k){
        x = k;
        y = k;
        z = k;
        yaw = k;
        return *this;
    }

    Point4d& operator=(int k){
        x = k;
        y = k;
        z = k;
        yaw = k;
        return *this;
    }

    Point4d operator*(Point4d k){
        return Point4d(x*k.x,y*k.y,z*k.z,yaw*k.yaw);
    }

    Point4d operator+(Point4d k){
        return Point4d(x+k.x,y+k.y,z+k.z,yaw+k.yaw);
    }

    Point4d& operator*=(cv::Point3d &p){
        x*=p.x;
        y*=p.x;
        z*=p.y;
        yaw*=p.z;
        return *this;
    }

    Point4d& operator+=(double k){
        x +=k;
        y +=k;
        z +=k;
        yaw +=k;
        return *this;
    }

    Point4d& operator+=(Point4d k){
        x +=k.x;
        y +=k.y;
        z +=k.z;
        yaw +=k.yaw;
        return *this;
    }

    Point4d& operator/=(Point4d k){
        x /=k.x;
        y /=k.y;
        z /=k.z;
        yaw /=k.yaw;
        return *this;
    }

    Point4d operator -(Point4d k){
        return Point4d(x-k.x,y-k.y,z-k.z,yaw-k.yaw);
    }

    bool is_nan(){
        return isnan(x) || isnan(y) || isnan(z) || isnan(yaw);
    }

   double x,y,z,yaw;
};


class Controller
{
public:
    Controller() :
                    _p(0),
                    _i(0),
                    _d(0),
                    _integrator(0),
                    _last_input(0),
                    _error(0),
                    _dInput(0),
                    _last_derivative(0),
                    _d_lpf_alpha(PID_D_TERM_FILTER),
                    _active(ENABLED),
                    lastUpdate(0)
    {
    }

    Controller(Const_PID x,Const_PID y, Const_PID range, Const_PID yaw,double lpf) :
                    _p(0),
                    _i(0),
                    _d(0),
                    _pid_output(0),
                    _last_pid_output(0),
                    LPF(lpf),
                    _integrator(0),
                    _last_input(0),
                    _error(0),
                    _dInput(0),
                    _last_derivative(0),
                    _d_lpf_alpha(PID_D_TERM_FILTER),
                    _active(ENABLED),
                    lastUpdate(0)
    {
        cX = x;
        cY = y;
        cRange = range;
        cYaw = yaw;


    }

    Point4d get_pid(Point4d targetInput, Point4d actualInput, float dt);
    Point4d get_pi(Point4d error);
    Point4d get_p(Point4d error);
    Point4d get_i(Point4d error, float dt);
    Point4d get_d(Point4d input, float dt);
    Point4d get_error();
    Point4d get_p_error_lim(Point4d targetInput, Point4d actualInput);

    void reset_I();
    Point4d get_integrator();

    int get_mode();
    void set_active(bool activate);
    bool get_active();

    float get_pMax();
    float get_iMax();

    bool is_k_zero();
    float get_kp();
    float get_ki();
    float get_kd();
    double get_mav_kp();
    double get_mav_ki();
    double get_mav_kd();
    bool is_k_changed();
    void set_k_pid(Const_PID x,Const_PID y, Const_PID range, Const_PID yaw,double lpf);
    double set_d_lpf_alpha(int cutoff_frequency, float time_step);

    Point4d get_p_term();
    Point4d get_i_term();
    Point4d get_d_term();

    Const_PID cX,cY,cRange,cYaw;

private:
    void Initialize();



    Point4d _p, _i, _d;
    Point4d _pid_output,_last_pid_output;
    double LPF;

    Point4d _integrator;
    Point4d _last_input;
    Point4d _error, _last_error;
    Point4d _dInput;
    Point4d _last_derivative;
    float _d_lpf_alpha;

    bool _active;
    unsigned long lastUpdate;

};
