#include "AP_Winch.h"

#if AP_WINCH_ENABLED

#include "AP_Winch_PWM.h"
#include "AP_Winch_Daiwa.h"
#include "AP_Winch_Mavlink.h"

#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_Winch::var_info[] = {
    // 0 was ENABLE

    // @Param: _TYPE
    // @DisplayName: Winch Type
    // @Description: Winch Type
    // @User: Standard
    // @Values: 0:None, 1:PWM, 2:Daiwa
    AP_GROUPINFO_FLAGS("_TYPE", 1, AP_Winch, config.type, (int8_t)WinchType::NONE, AP_PARAM_FLAG_ENABLE),

    // @Param: _RATE_MAX
    // @DisplayName: Winch deploy or retract rate maximum
    // @Description: Winch deploy or retract rate maximum.  Set to maximum rate with no load.
    // @User: Standard
    // @Range: 0 10
    // @Units: m/s
    AP_GROUPINFO("_RATE_MAX", 2, AP_Winch, config.rate_max, 1.0f),

    // @Param: _POS_P
    // @DisplayName: Winch control position error P gain
    // @Description: Winch control position error P gain
    // @Range: 0.01 10.0
    // @User: Standard
    AP_GROUPINFO("_POS_P", 3, AP_Winch, config.pos_p, 1.0f),

    // @Param: _OPTIONS
    // @DisplayName: Winch control options
    // @Description: Winch control options
    // @Bitmask: 0:GroundSensse
    // @User: Standard
    AP_GROUPINFO("_OPTIONS", 4, AP_Winch, config.options, 0),

    // 4 was _RATE_PID

    AP_GROUPEND
};

AP_Winch::AP_Winch()
{
    if (_singleton) {
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        AP_HAL::panic("Too many winches");
#endif
        return;
    }
    _singleton = this;

    AP_Param::setup_object_defaults(this, var_info);
}

// indicate whether this module is enabled
bool AP_Winch::enabled() const
{
   return ((config.type > 0) && (backend != nullptr));
}

// true if winch is healthy
bool AP_Winch::healthy() const
{
    if (backend != nullptr) {
        return backend->healthy();
    }
    return false;
}

void AP_Winch::init()
{
    switch ((WinchType)config.type.get()) {
    case WinchType::NONE:
        break;
    case WinchType::PWM:
        backend = new AP_Winch_PWM(config);
        break;
    case WinchType::DAIWA:
        backend = new AP_Winch_Daiwa(config);
        break;
    case WinchType::MAVLINK:
        backend = new AP_Winch_Mavlink(config);
        break;
    default:
        break;
    }
    if (backend != nullptr) {
        backend->init();
    }
}

// release specified length of cable (in meters)
void AP_Winch::release_length(float length)
{
    if (backend == nullptr) {
        return;
    }
    if ((WinchType)config.type.get() == WinchType::MAVLINK)
        config.length_relative=length;
    config.length_desired = backend->get_current_length() + length;
    config.control_mode = ControlMode::POSITION;
    backend->command_time=AP_HAL::millis();
}

// deploy line at specified speed in m/s (+ve deploys line, -ve retracts line, 0 stops)
void AP_Winch::set_desired_rate(float rate, bool auto_mode)
{
    config.rate_desired = constrain_float(rate, -get_rate_max(), get_rate_max());
    config.control_mode = ControlMode::RATE;
    if(auto_mode==true)
        backend->command_time=AP_HAL::millis();

}

// relax the winch so it does not attempt to maintain length or rate
void AP_Winch::relax(bool auto_mode) 
{ 
    config.control_mode = ControlMode::RELAXED; 
    if(auto_mode==true)
        backend->command_time=AP_HAL::millis();
}

// put the winch into delivery mode
void AP_Winch::deliver() 
{ 
    config.control_mode = ControlMode::DELIVER; 
    backend->command_time=AP_HAL::millis();
}

// put the winch into delivery mode
void AP_Winch::retract() 
{ 
    config.control_mode = ControlMode::RETRACT; 
    config.length_desired=0; 
    backend->command_time=AP_HAL::millis();
}

// send status to ground station
void AP_Winch::send_status(const GCS_MAVLINK &channel)
{
    if (backend != nullptr) {
        backend->send_status(channel);
    }
}

// returns true if pre arm checks have passed
bool AP_Winch::pre_arm_check(char *failmsg, uint8_t failmsg_len) const
{
    // succeed if winch is disabled
    if ((WinchType)config.type.get() == WinchType::NONE) {
        return true;
    }

    // fail if unhealthy
    if (!healthy() && (WinchType)config.type.get() != WinchType::MAVLINK) {
        hal.util->snprintf(failmsg, failmsg_len, "winch unhealthy");
        return false;
    }

    return true;
}

void AP_Winch::handleMessage(const mavlink_message_t& msg)
{
    if ((WinchType)config.type.get() == WinchType::MAVLINK && backend!=nullptr)
        {
            mavlink_winch_status_t Winch;
            mavlink_msg_winch_status_decode(&msg,&Winch);
            backend->handleMessage(Winch);
        }
}

void AP_Winch::handle_command_ack(const mavlink_message_t& msg)
{
    if ((WinchType)config.type.get() == WinchType::MAVLINK && backend!=nullptr)
        {
            mavlink_command_ack_t Ack;
            mavlink_msg_command_ack_decode(&msg,&Ack);
            backend->handle_command_ack(Ack);
        }
}

int AP_Winch::ground_sense()
{
    if(backend!=nullptr)
        return backend->ground_sense();
        else
        return -1;

}

float AP_Winch::get_line_length()
{
    if(backend!=nullptr)
        return backend->get_line_length();
        else
        return -1;

}

// update - should be called at at least 10hz
#define PASS_TO_BACKEND(function_name) \
    void AP_Winch::function_name()   \
    {                                  \
        if (!enabled()) {              \
            return;                    \
        }                              \
        if (backend != nullptr) {      \
            backend->function_name();  \
        }                              \
    }

PASS_TO_BACKEND(update)
PASS_TO_BACKEND(write_log)

#undef PASS_TO_BACKEND

/*
 * Get the AP_Winch singleton
 */
AP_Winch *AP_Winch::_singleton;
AP_Winch *AP_Winch::get_singleton()
{
    return _singleton;
}

namespace AP {

AP_Winch *winch()
{
    return AP_Winch::get_singleton();
}

};

#endif  // AP_WINCH_ENABLED
