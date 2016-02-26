// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/// @file	RC_Channel.h
/// @brief	RC_Channel manager, with EEPROM-backed storage of constants.

#ifndef __RC_CHANNEL_H__
#define __RC_CHANNEL_H__

#define RC_CHANNEL_TYPE_ANGLE       0
#define RC_CHANNEL_TYPE_RANGE       1
#define RC_CHANNEL_TYPE_ANGLE_RAW   2

#define RC_MAX_CHANNELS 14

/// @class	RC_Channel
/// @brief	Object managing one RC channel
class RC_Channel {
public:
    /// Constructor
    ///
    /// @param key      EEPROM storage key for the channel trim parameters.
    /// @param name     Optional name for the group.
    ///
    RC_Channel() :
        _high_out(1),
        radio_min(1100),radio_max(1900),radio_trim(1500),_reverse(1)
    {

    }

    // used to get min/max/trim limit value based on _reverse
    enum LimitValue {
        RC_CHANNEL_LIMIT_TRIM,
        RC_CHANNEL_LIMIT_MIN,
        RC_CHANNEL_LIMIT_MAX
    };

    struct Property{
        long max;
        long min;
        long trim;
    };


    // startup
    void        set_type(unsigned int t);

    // setup the control preferences
    void        set_range(long low, long high);
    void        get_range(long &low_out, long &high_out);
    void        set_angle(long angle);
    void        set_reverse(bool reverse);
    bool        get_reverse(void) const;
    void        set_trim(long trim);
    void        set_property(Property p);
    Property    get_property();
    
    // get the center stick position expressed as a control_in value
    long        get_control_mid() const;

    // return a limit PWM value
    unsigned long    get_limit_pwm(LimitValue limit) const;

    // generate PWM from servo_out value
    long        calc_pwm(float val);

    long        radio_min;
    long        radio_trim;
    long        radio_max;
    float       input;
    float       output;

    // includes offset from PWM
    //long   get_radio_out(void);

    /*
      return a normalised input for a channel, in range -1 to 1,
      centered around the channel trim. Ignore deadzone.
     */
    float                                           norm_input();

    /*
      return a normalised input for a channel, in range -1 to 1,
      centered around the channel trim. Take into account the deadzone
    */

    unsigned int                                    percent_input();
    float                                           norm_output();
    long                                         angle_to_pwm();
    long                                         range_to_pwm();


private:
    unsigned int         _reverse;
    unsigned int         _type;
    long         _high_out;
    long         _low_out;

};


#endif
