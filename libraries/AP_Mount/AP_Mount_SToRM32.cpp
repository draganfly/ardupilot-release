#include "AP_Mount_SToRM32.h"
#if HAL_MOUNT_ENABLED
#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

AP_Mount_SToRM32::AP_Mount_SToRM32(AP_Mount &frontend, AP_Mount::mount_state &state, uint8_t instance) :
    AP_Mount_Backend(frontend, state, instance),
    _chan(MAVLINK_COMM_0)
{
    _query=false;
    _last_gimbal_info_request=0;
    _gimbal_options_mask=0;
    _rate_output=false;
    _require_change_mode=false;
    _force_change=false;
}

// update mount position - should be called periodically
void AP_Mount_SToRM32::update()
{
    // exit immediately if not initialised
    if (!_initialised) {
        find_gimbal();
        return;
    }
    if(!_query)
    {
        query_gimbal();
    }
    else
    {
        if((_require_change_mode==true && (AP_HAL::millis()-_mode_change_timer)>=AP_MOUNT_STORM32_COMMAND_RETRY_MS) || (AP_HAL::millis()-_mode_change_timer)>=AP_MOUNT_STORM32_RESEND_MODE_MS)
            {
            _mode_change_timer=AP_HAL::millis();
            float input_mode=0;
            if(_rate_output==true)
                input_mode=1;
            mavlink_msg_command_long_send(_chan,_sysid,_compid,MAV_CMD_DO_MOUNT_CONFIGURE,0,get_mode(),0,0,0,input_mode,input_mode,input_mode);
            }

    }
    

    // flag to trigger sending target angles to gimbal
    bool resend_now = false;
    bool block_send=false;

    // update based on mount mode
    MAV_MOUNT_MODE mode=get_mode();
    switch(mode) {
        // move mount to a "retracted" position.  To-Do: remove support and replace with a relaxed mode?
        case MAV_MOUNT_MODE_RETRACT:
            {
            const Vector3f &target = _state._retract_angles.get();
            _angle_ef_target_rad.x = ToRad(target.x);
            _angle_ef_target_rad.y = ToRad(target.y);
            _angle_ef_target_rad.z = ToRad(target.z);
            }
            break;

        // move mount to a neutral position, typically pointing forward
        case MAV_MOUNT_MODE_NEUTRAL:
            {
            const Vector3f &target = _state._neutral_angles.get();
            _angle_ef_target_rad.x = ToRad(target.x);
            _angle_ef_target_rad.y = ToRad(target.y);
            _angle_ef_target_rad.z = ToRad(target.z);
            }
            break;

        // point to the angles given by a mavlink message
        case MAV_MOUNT_MODE_MAVLINK_TARGETING:
            if(_query && _gimbal_options_mask&GIMBAL_MODE_AWARE_MASK)
                {
                if(_last_mode!=MAV_MOUNT_MODE_MAVLINK_TARGETING || _force_change==true)
                    {
                    _require_change_mode=true;
                    _mount_ack_control=false;
                    _mode_change_timer=0;
                    _mount_control_timer=0;
                    _force_change=false;
                    }
                _rate_output=false;
                if(fabsf(ToDeg(_angle_ef_target_rad.y)-_last_tilt)>0.01f || fabsf(ToDeg(_angle_ef_target_rad.x)-_last_roll)>0.01f || fabsf(ToDeg(_angle_ef_target_rad.z) -_last_pan)>0.01f)
                    _mount_ack_control=false;
                if(_require_change_mode==false)
                    {
                    if((AP_HAL::millis()-_mount_control_timer)>=AP_MOUNT_STORM32_COMMAND_RETRY_MS && _mount_ack_control==false)
                        {
                        send_do_mount_control(-ToDeg(_angle_ef_target_rad.y), ToDeg(_angle_ef_target_rad.x), ToDeg(_angle_ef_target_rad.z), mode); 
                        _last_tilt=ToDeg(_angle_ef_target_rad.y);
                        _last_roll=ToDeg(_angle_ef_target_rad.x);
                        _last_pan=ToDeg(_angle_ef_target_rad.z);
                        }
                    }
                block_send=true;
                }
                else
                {
                // do nothing because earth-frame angle targets (i.e. _angle_ef_target_rad) should have already been set by a MOUNT_CONTROL message from GCS
                resend_now = true;
                }
            break;

        // RC radio manual angle control, but with stabilization from the AHRS
        case MAV_MOUNT_MODE_RC_TARGETING:
            // update targets using pilot's rc inputs
            if(_query && _gimbal_options_mask&GIMBAL_MODE_AWARE_MASK)
                {
                block_send=true;
                if(_last_mode!=MAV_MOUNT_MODE_RC_TARGETING || _force_change==true)
                    {
                    _require_change_mode=true;
                    _mode_change_timer=0;
                    _force_change=false;
                    }
                if(_gimbal_options_mask&GIMBAL_PREFER_RATE_CONTROL_MASK)
                    _rate_output=true;
                    else
                    _rate_output=false;
                if(_require_change_mode==false)
                    {
                    if(_rate_output==true)
                        {
                        float roll,tilt,pan;
                        get_rate_control(roll,tilt,pan);
                        send_do_mount_control(tilt,roll,pan,mode);
                        }
                        else
                        {
                        update_targets_from_rc();
                        block_send=false;
                        resend_now = true;       
                        }
                    }
                    
                }
                else
                {
                update_targets_from_rc();
                resend_now = true;
                }
            break;

        // point mount to a GPS point given by the mission planner
        case MAV_MOUNT_MODE_GPS_POINT:
            if(_query && _gimbal_options_mask&GIMBAL_NATIVE_GPS_POINT_MASK)
                {
                block_send=true;
                if(_last_mode!=MAV_MOUNT_MODE_GPS_POINT)
                    {
                    _mount_control_timer=0;
                    _mount_ack_control=false;
                    }
                if(!_last_roi_target.same_latlon_as(_state._roi_target))
                    _mount_ack_control=false;
                if((AP_HAL::millis()-_mount_control_timer)>=AP_MOUNT_STORM32_COMMAND_RETRY_MS && _mount_ack_control==false)
                    {
                    int32_t alt,lat,lon;
                    if(get_roi(alt,lat,lon)==true)
                        mavlink_msg_command_long_send(_chan,_sysid,_compid,MAV_CMD_DO_MOUNT_CONTROL,0,0,0,0,(float)alt/1E3,(float)lat/1E7,(float)lon/1E7,mode); 
                        else
                        set_mode(MAV_MOUNT_MODE_RC_TARGETING);
                    _last_roi_target=_state._roi_target; 
                    }
                }
                else
                {
                if (calc_angle_to_roi_target(_angle_ef_target_rad, true, true)) {
                    resend_now = true;
                    }
                }
                

            break;

        case MAV_MOUNT_MODE_HOME_LOCATION:
            // constantly update the home location:
            if (!AP::ahrs().home_is_set()) {
                break;
            }
            _state._roi_target = AP::ahrs().get_home();
            _state._roi_target_set = true;

            
            if(_query && _gimbal_options_mask&GIMBAL_NATIVE_GPS_POINT_MASK)
                {
                block_send=true;
                if(_last_mode!=MAV_MOUNT_MODE_HOME_LOCATION)
                    {
                    _mount_control_timer=0;
                    _mount_ack_control=false;
                    }
                if((AP_HAL::millis()-_mount_control_timer)>=AP_MOUNT_STORM32_COMMAND_RETRY_MS && _mount_ack_control==false)
                    {
                    int32_t alt,lat,lon;
                    if(get_roi(alt,lat,lon)==true)
                        mavlink_msg_command_long_send(_chan,_sysid,_compid,MAV_CMD_DO_MOUNT_CONTROL,0,0,0,0,(float)alt/1E3,(float)lat/1E7,(float)lon/1E7,mode); 
                        else
                        set_mode(MAV_MOUNT_MODE_RC_TARGETING);
                        
                    }
                }
                else
                {
                if (calc_angle_to_roi_target(_angle_ef_target_rad, true, true)) {
                    resend_now = true;
                    }
                }
            break;

        case MAV_MOUNT_MODE_SYSID_TARGET:
            if(_query && _gimbal_options_mask&GIMBAL_NATIVE_GPS_POINT_MASK)
                {
                block_send=true;
                int32_t alt,lat,lon;
                if(get_roi(alt,lat,lon)==true)
                    mavlink_msg_command_long_send(_chan,_sysid,_compid,MAV_CMD_DO_MOUNT_CONTROL,0,0,0,0,(float)alt/1E3,(float)lat/1E7,(float)lon/1E7,mode); 
                    else
                    set_mode(MAV_MOUNT_MODE_RC_TARGETING);
                }
                else
                {
                if (calc_angle_to_roi_target(_angle_ef_target_rad, true, true)) {
                    resend_now = true;
                    }
                }
            break;

        default:
            // we do not know this mode so do nothing
            break;
    }
    _last_mode=mode;

    // resend target angles at least once per second
    if ((resend_now || ((AP_HAL::millis() - _last_send) > AP_MOUNT_STORM32_RESEND_MS)) && block_send==false) {
        MAV_MOUNT_MODE send_mode=MAV_MOUNT_MODE_MAVLINK_TARGETING;
        if(_gimbal_options_mask&GIMBAL_MODE_AWARE_MASK)
            send_mode=get_mode();
        send_do_mount_control(ToDeg(_angle_ef_target_rad.y), ToDeg(_angle_ef_target_rad.x), ToDeg(_angle_ef_target_rad.z), send_mode);
    }
}

// has_pan_control - returns true if this mount can control it's pan (required for multicopters)
bool AP_Mount_SToRM32::has_pan_control() const
{
    // we do not have yaw control
    return (_gimbal_options_mask&GIMBAL_CONTINUOUS_PAN_MASK);
}

// set_mode - sets mount's mode
void AP_Mount_SToRM32::set_mode(enum MAV_MOUNT_MODE mode)
{
    // exit immediately if not initialised
    if (!_initialised) {
        return;
    }

    // record the mode change
    _state._mode = mode;
}

// send_mount_status - called to allow mounts to send their status to GCS using the MOUNT_STATUS message
void AP_Mount_SToRM32::send_mount_status(mavlink_channel_t chan)
{
    // return target angles as gimbal's actual attitude.  To-Do: retrieve actual gimbal attitude and send these instead
    mavlink_msg_mount_status_send(chan, 0, 0, ToDeg(_angle_ef_target_rad.y)*100, ToDeg(_angle_ef_target_rad.x)*100, ToDeg(_angle_ef_target_rad.z)*100, _state._mode);
}

// search for gimbal in GCS_MAVLink routing table
void AP_Mount_SToRM32::find_gimbal()
{
    // return immediately if initialised
    if (_initialised) {
        return;
    }

    // return if search time has has passed
    if (AP_HAL::millis() > AP_MOUNT_STORM32_SEARCH_MS) {
        return;
    }

    if (GCS_MAVLINK::find_by_mavtype(MAV_TYPE_GIMBAL, _sysid, _compid, _chan)) {
        _initialised = true;
    }
}

// attempt to get an GIMBAL_DEVICE_INFORMATION message
void AP_Mount_SToRM32::query_gimbal()
{
    // return if search time has has passed
    if (AP_HAL::millis() > AP_MOUNT_STORM32_SEARCH_MS) {
        return;
    }

    if(_compid!=26) //dirty hack so this only attempts to talk to the QA board, gremsy gimbals seem to be broken by the below call
        return; 

    if(((AP_HAL::millis() - _last_gimbal_info_request) > AP_MOUNT_STORM32_RESEND_QUERY_MS))
        {
        mavlink_msg_param_request_read_send(_chan,_sysid,_compid,AP_MOUNT_GIMBAL_OPTIONS_MASK_NAME,-1);
        _last_gimbal_info_request=AP_HAL::millis();
        }
}

                       
void AP_Mount_SToRM32::handle_param_value(const mavlink_message_t &msg)
{
    mavlink_param_value_t param_value;
    if(msg.msgid != MAVLINK_MSG_ID_PARAM_VALUE || msg.sysid!=_sysid || msg.compid!=_compid)
        return;
    mavlink_msg_param_value_decode(&msg,&param_value);
    if(strncmp(AP_MOUNT_GIMBAL_OPTIONS_MASK_NAME,param_value.param_id,16)==0)
        {
        memcpy(&_gimbal_options_mask,&param_value.param_value,4);
        _query=true;
        _force_change=true;
        }
}

void AP_Mount_SToRM32::handle_command_ack(const mavlink_message_t &msg)
{
    mavlink_command_ack_t ack;
    if(msg.msgid != MAVLINK_MSG_ID_COMMAND_ACK || msg.sysid!=_sysid || msg.compid!=_compid)
        return;  
    mavlink_msg_command_ack_decode(&msg,&ack);
    if(ack.command == MAV_CMD_DO_MOUNT_CONFIGURE && ack.result==MAV_RESULT_ACCEPTED)
        _require_change_mode=false;
    if(ack.command == MAV_CMD_DO_MOUNT_CONTROL && ack.result==MAV_RESULT_ACCEPTED)
        _mount_ack_control=true;


}

// send_do_mount_control - send a COMMAND_LONG containing a do_mount_control message
void AP_Mount_SToRM32::send_do_mount_control(float pitch_deg, float roll_deg, float yaw_deg, enum MAV_MOUNT_MODE mount_mode)
{
    // exit immediately if not initialised
    if (!_initialised) {
        return;
    }

    // check we have space for the message
    if (!HAVE_PAYLOAD_SPACE(_chan, COMMAND_LONG)) {
        return;
    }

    // reverse pitch and yaw control
    pitch_deg = -pitch_deg;
    yaw_deg = -yaw_deg;

    // send command_long command containing a do_mount_control command
    mavlink_msg_command_long_send(_chan,
                                  _sysid,
                                  _compid,
                                  MAV_CMD_DO_MOUNT_CONTROL,
                                  0,        // confirmation of zero means this is the first time this message has been sent
                                  pitch_deg,
                                  roll_deg,
                                  yaw_deg,
                                  0, 0, 0,  // param4 ~ param6 unused
                                  mount_mode);

    // store time of send
    _last_send = AP_HAL::millis();
}
#endif // HAL_MOUNT_ENABLED
