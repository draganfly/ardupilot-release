#include <AP_Winch/AP_Winch_Mavlink.h>

#include <AP_Logger/AP_Logger.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

// true if winch is healthy
bool AP_Winch_Mavlink::healthy() const
{
    // healthy if we have received data within 3 seconds
    return (AP_HAL::millis() - data_update_ms < 3000);
}

void AP_Winch_Mavlink::init()
{
    // call superclass init
    config.length_desired=0;
    config.length_relative=0;
    config.rate_desired=0;
    config.control_mode=AP_Winch::ControlMode::RATE;
    last_mode=(uint8_t)config.control_mode;
    last_rate_desired=config.rate_desired;
    last_length_desired=config.length_desired;
    command_acked=true;
    AP_Winch_Backend::init();
}

void AP_Winch_Mavlink::update()
{
    // return immediately if no servo is assigned to control the winch
    if (SRV_Channels::function_assigned(SRV_Channel::k_winch)) {
        read_pilot_desired_rate();
    }

    // read pilot input


    // send outputs to winch
    control_winch();
}

//send generator status
void AP_Winch_Mavlink::send_status(const GCS_MAVLINK &channel)
{
    //do nothing since the winch_status message is broadcast and should be routed to all mavlink devices
}

// write log
void AP_Winch_Mavlink::write_log()
{
    uint16_t tension=0;
    if(!isnan(Status.tension))
        tension=(uint16_t)fabsf(Status.tension);
    AP::logger().Write_Winch(
            healthy(),
            (Status.status&MAV_WINCH_STATUS_FULLY_RETRACTED)!=0,
            (Status.status&MAV_WINCH_STATUS_MOVING)!=0,
            (Status.status&MAV_WINCH_STATUS_CLUTCH_ENGAGED)!=0,
            (uint8_t)config.control_mode,
            config.length_desired,
            Status.line_length,
            config.rate_desired,
            tension,
            Status.voltage,
            (int8_t)Status.temperature);
}


// update pwm outputs to control winch
void AP_Winch_Mavlink::control_winch()
{
    uint8_t sys_id, comp_id;
    mavlink_channel_t chan;
    if(!GCS_MAVLINK::find_by_mavtype(MAV_TYPE_WINCH,sys_id,comp_id,chan))
        return;
    //check if something has changed to last command was not acked
    if(last_mode!=(uint8_t)config.control_mode ||
         (fabsf(last_rate_desired-config.rate_desired)>0.001f && config.control_mode==AP_Winch::ControlMode::RATE) ||
         (fabsf(last_rate_desired-config.rate_desired)>0.001f && config.control_mode==AP_Winch::ControlMode::RATE_FROM_RC) ||
         (fabsf(last_length_desired-config.length_desired)>0.001f && config.control_mode==AP_Winch::ControlMode::POSITION) ||
         ((AP_HAL::millis()-command_send_time)>WINCH_MAV_COMMAND_TIMEOUT && command_acked==false) ||
         ((command_time-last_command_time)>WINCH_COMMAND_REPEAT_INTERVAL))
        {
        last_command_time=command_time;
        command_acked=false;
        // check we have space for the message
        if (!HAVE_PAYLOAD_SPACE(chan, COMMAND_LONG)) {
            return;
        }
        float Param2=(float)WINCH_RATE_CONTROL;
        switch(config.control_mode)
            {
                case AP_Winch::ControlMode::DELIVER:
                    Param2=(float)WINCH_DELIVER;
                    break;
                case AP_Winch::ControlMode::POSITION:
                    Param2=(float)WINCH_RELATIVE_LENGTH_CONTROL;
                    break;
                case AP_Winch::ControlMode::RATE_FROM_RC:
                case AP_Winch::ControlMode::RATE:
                    Param2=(float)WINCH_RATE_CONTROL;
                    break;
                case AP_Winch::ControlMode::RELAXED:
                    Param2=(float)WINCH_RELAXED;
                    break;
                case AP_Winch::ControlMode::RETRACT:
                    Param2=(float)WINCH_RETRACT;
                    break;
            }
        // send command_long command containing a DO_WINCH
        mavlink_msg_command_long_send(chan,
                                    sys_id,
                                    comp_id,
                                    MAV_CMD_DO_WINCH,
                                    0,        // confirmation of zero means this is the first time this message has been sent
                                    1,
                                    Param2,
                                    config.length_relative,
                                    config.rate_desired,
                                    0, 0, 0
                                    );

        // store time of send
        command_send_time = AP_HAL::millis();
        last_mode=(uint8_t)config.control_mode;
        last_rate_desired=config.rate_desired;
        last_length_desired=config.length_desired;

        }


    /*
    const uint32_t now_ms = AP_HAL::millis();
    float dt = (now_ms - control_update_ms) * 0.001f;
    if (dt > 1.0f) {
        dt = 0.0f;
    }
    control_update_ms = now_ms;

    // if real doing any control output trim value
    if (config.control_mode == AP_Winch::ControlMode::RELAXED) {
        // if not doing any control output release clutch and move winch to trim
        SRV_Channels::set_output_limit(SRV_Channel::k_winch_clutch, SRV_Channel::Limit::MAX);
        SRV_Channels::set_output_scaled(SRV_Channel::k_winch, 0);

        // rate used for acceleration limiting reset to zero
        set_previous_rate(0.0f);
        return;
    }

    // release clutch
    SRV_Channels::set_output_limit(SRV_Channel::k_winch_clutch, SRV_Channel::Limit::MIN);

    // if doing position control, calculate position error to desired rate
    if ((config.control_mode == AP_Winch::ControlMode::POSITION) && healthy()) {
        float position_error = config.length_desired - latest.line_length;
        config.rate_desired = constrain_float(position_error * config.pos_p, -config.rate_max, config.rate_max);
    }

    // apply acceleration limited to rate
    const float rate_limited = get_rate_limited_by_accel(config.rate_desired, dt);

    // use linear interpolation to calculate output to move winch at desired rate
    float scaled_output = 0;
    if (!is_zero(rate_limited)) {
        scaled_output = linear_interpolate(output_dz, 1000, fabsf(rate_limited), 0, config.rate_max) * (is_positive(rate_limited) ? 1.0f : -1.0f);
    }
    SRV_Channels::set_output_scaled(SRV_Channel::k_winch, scaled_output);
    */
}


void AP_Winch_Mavlink::handleMessage(const mavlink_winch_status_t &msg)
{
    data_update_ms = AP_HAL::millis();
    Status=msg;
}

void AP_Winch_Mavlink::handle_command_ack(const mavlink_command_ack_t &msg)
{
    if(msg.command==MAV_CMD_DO_WINCH)
        command_acked=true;
}

int AP_Winch_Mavlink::ground_sense()
{
    if(healthy())
        {
            return (Status.status&(1<<7))!=0;    //this should be MAV_WINCH_STATUS_GROUND_SENSE once the mavlink headers get updated
        }
    return -1;
}

    //get current line length
float AP_Winch_Mavlink::get_line_length() 
{
if(healthy())
    return Status.line_length;
    else
    return 0;    
}