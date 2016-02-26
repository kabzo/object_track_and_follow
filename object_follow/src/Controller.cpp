#include "Controller.h"


bool Controller::is_k_changed()
{
    //if (_kP.is_changed() || _kD.is_changed() || _kI.is_changed())
    //    return true;
    return false;
}

void Controller::set_k_pid(Const_PID x,Const_PID y, Const_PID range, Const_PID yaw,double lpf)
{
    cX = x;
    cY = y;
    cRange = range;
    cYaw = yaw;
    LPF = lpf;
}

Point4d Controller::get_p(Point4d error)
{
    Point4d p;
    p = error * Point4d(cX.cP,cY.cP,cRange.cP,cYaw.cP);
    p = p.limit(cX.pLim,cY.pLim,cRange.pLim,cYaw.pLim);
    return p;
}

Point4d Controller::get_i(Point4d error, float dt)
{
    if (cX.cI!=0 || cY.cI != 0 || cRange.cI != 0 || cYaw.cI != 0)
    {
        float _deltaTimeSec = dt / 1000.0;
        Point4d e = error * Point4d(cX.cI,cY.cI,cRange.cI,cYaw.cI);
        _integrator += ((Point4d) e) * _deltaTimeSec;

       _integrator.limit(cX.iLim,cY.iLim,cRange.iLim,cYaw.iLim);
        return _integrator;
    }
    return Point4d(0);
}

Point4d Controller::get_d(Point4d input, float dt)
{
    if ((cX.cD!=0 || cY.cD != 0 || cRange.cD != 0 || cYaw.cD != 0))
    {
        Point4d derivative;
        if (_last_derivative.is_nan())
        {
            // we've just done a reset, suppress the first derivative
            // term as we don't want a sudden change in input to cause
            // a large D output change
            derivative = 0;
            _last_derivative = 0;
        } else
        {
            double _deltaTimeSec = dt / 1000.0;
            derivative = (input - _last_input) ;
            derivative/=_deltaTimeSec;

            //calculate instantaneous derivative
        }

        //	discrete low pass filter, cuts out the
        //	high frequency noise that can drive the controller crazy
        // see wikipedia low pass filter
        derivative = _last_derivative +  (derivative - _last_derivative)*((double)_d_lpf_alpha);
        _last_derivative = derivative;
        _last_input = input;
        return derivative* Point4d(-cX.cD,-cY.cD,-cRange.cD,-cYaw.cD);
    }
    return Point4d(0);
}

Point4d Controller::get_pi(Point4d error)
{
    return get_p(error) + get_i(error, 0);
}

Point4d Controller::get_pid(Point4d targetInput, Point4d actualInput, float dt)
{
    if (_active == DISABLED)
        return Point4d(0);

    _error = targetInput - actualInput;
    _p = get_p(_error);
    _i = get_i(_error, dt);
    _d = get_d(actualInput, dt);

    _pid_output = _p+_i+_d;
    return _pid_output;			//, -_outLimit._value, _outLimit._value);
}

Point4d Controller::get_error(){
    return _error;
}

Point4d Controller::get_p_error_lim(Point4d targetInput, Point4d actualInput){
    _error = targetInput - actualInput;
  _error.limit(cX.pLim,cY.pLim,cRange.pLim,cYaw.pLim);
    return _error;
}

double Controller::set_d_lpf_alpha(int cutoff_frequency, float time_step)
{
    // calculate alpha
    float rc = 1/(2*M_PI*cutoff_frequency);
    return time_step / (time_step + rc);



}

void Controller::reset_I()
{
    _integrator.x=_integrator.y = 0;
    // mark derivative as invalid
    _last_derivative.x = _last_derivative.y = NAN;
}

int Controller::get_mode()
{
    return _active ? ENABLED : DISABLED;
}


Point4d Controller::get_integrator()
{
    return _integrator;
}

void Controller::set_active(bool activate)
{
    _active = activate;
}

bool Controller::get_active()
{
    return _active;
}

Point4d Controller::get_p_term(){
    return _p;
}

Point4d Controller::get_i_term(){
    return _i;
}

Point4d Controller::get_d_term(){
    return _d;
}

