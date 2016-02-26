// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 *       RC_Channel.cpp - Radio library for Arduino
 *       Code by Jason Short. DIYDrones.com
 *
 */

#include <stdlib.h>

#include "RC_Channel.h"
#include <math.h>

#define constrain(x,min,max) x<min?min:(x>max?max:x)

// setup the control preferences
void
RC_Channel::set_range(long low, long high)
{
    _type           = RC_CHANNEL_TYPE_RANGE;
    _high_out       = high;
    _low_out        = low;
}

void       RC_Channel::get_range(long &low_out, long &high_out){
    low_out = _low_out;
    high_out = _high_out;
}


void
RC_Channel::set_angle(long angle)
{
    _type   = RC_CHANNEL_TYPE_ANGLE;
    _high_out   = angle;
}

void
RC_Channel::set_reverse(bool reverse)
{
    if (reverse) _reverse = -1;
    else _reverse = 1;
}

void RC_Channel::set_trim(long trim){
    radio_trim =trim;
}

void RC_Channel::set_property(RC_Channel::Property p){
    radio_max = p.max;
    radio_min = p.min;
    radio_trim = p.trim;
}


bool
RC_Channel::get_reverse(void) const
{
    if (_reverse == -1) {
        return true;
    }
    return false;
}

void
RC_Channel::set_type(unsigned int t)
{
    _type = t;
}

// returns just the PWM without the offservo_outset from radio_min
long
RC_Channel::calc_pwm(float val)
{
    input = val;
    long pwm;
    if(_type == RC_CHANNEL_TYPE_RANGE) {
        pwm         = range_to_pwm();
        output       = (_reverse >= 0) ? (radio_min + pwm) : (radio_max - pwm);

    }else if(_type == RC_CHANNEL_TYPE_ANGLE_RAW) {
        pwm         = (float)input * 0.1f;
        long reverse_mul = (_reverse==-1?-1:1);
        output       = (pwm * reverse_mul) + radio_trim;

    }else{     // RC_CHANNEL_TYPE_ANGLE
        pwm         = angle_to_pwm();
        output       = pwm + radio_trim;
    }

    return constrain(output, radio_min, radio_max);
}

/*
  return the center stick position expressed as a control_in value
  used for thr_mid in copter
 */
long
RC_Channel::get_control_mid() const {
    if (_type == RC_CHANNEL_TYPE_RANGE) {
        long r_in = (radio_min+radio_max)/2;

        if (_reverse == -1) {
            r_in = radio_max - (r_in - radio_min);
        }

        long radio_trim_low  = radio_min + 30;

        return (_low_out + ((int32_t)(_high_out - _low_out) * (int32_t)(r_in - radio_trim_low)) / (int32_t)(radio_max - radio_trim_low));
    } else {
        return 0;
    }
}

// ------------------------------------------
long
RC_Channel::angle_to_pwm()
{
    long reverse_mul = (_reverse==-1?-1:1);
    if((input * reverse_mul) > 0) {
        return round(reverse_mul * (input * (float)(radio_max - radio_trim)) / (float)_high_out);
    } else {
        return round(reverse_mul * (input * (float)(radio_trim - radio_min)) / (float)_high_out);
    }
}

RC_Channel::Property RC_Channel::get_property(){
    Property p;
    p.max = radio_max;
    p.min = radio_min;
    p.trim = radio_trim;
    return p;
}


long
RC_Channel::range_to_pwm()
{
    if (_high_out == _low_out) {
        return radio_trim;
    }
    return round(((input - _low_out) * (float)(radio_max - radio_min)) / (float)(_high_out - _low_out)) ;
}

// ------------------------------------------
float
RC_Channel::norm_input()
{
    float ret;
    long reverse_mul = (_reverse==-1?-1:1);
    if (input < radio_trim) {
        ret = reverse_mul * (float)(input - radio_trim) / (float)(radio_trim - radio_min);
    } else {
        ret = reverse_mul * (float)(input - radio_trim) / (float)(radio_max  - radio_trim);
    }
    return constrain(ret, -1.0f, 1.0f);
}

/*
  get percentage input from 0 to 100. This ignores the trim value.
 */
unsigned int
RC_Channel::percent_input()
{
    if (output <= radio_min) {
        return _reverse==-1?100:0;
    }
    if (output >= radio_max) {
        return _reverse==-1?0:100;
    }
    unsigned int ret = 100.0f * (output - radio_min) / (float)(radio_max - radio_min);
    if (_reverse == -1) {
        ret = 100 - ret;
    }
    return ret;
}

float
RC_Channel::norm_output()
{
    long mid = (radio_max + radio_min) / 2;
    float ret;
    if (mid <= radio_min) {
        return 0;
    }
    if (output < mid) {
        ret = (float)(output - mid) / (float)(mid - radio_min);
    } else if (output > mid) {
        ret = (float)(output - mid) / (float)(radio_max  - mid);
    } else {
        ret = 0;
    }
    if (_reverse == -1) {
	    ret = -ret;
    }
    return ret;
}

// return a limit PWM value
ulong RC_Channel::get_limit_pwm(LimitValue limit) const
{
    switch (limit) {
    case RC_CHANNEL_LIMIT_TRIM:
        return radio_trim;
    case RC_CHANNEL_LIMIT_MAX:
        return get_reverse() ? radio_min : radio_max;
    case RC_CHANNEL_LIMIT_MIN:
        return get_reverse() ? radio_max : radio_min;
    }
    // invalid limit value, return trim
    return radio_trim;
}
