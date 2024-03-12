#include "AP_Mount_SToRM32.h"

#if HAL_MOUNT_STORM32MAVLINK_ENABLED
#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

#define AP_MOUNT_STORM32_RESEND_MS  1000    // resend angle targets to gimbal once per second
#define AP_MOUNT_STORM32_SEARCH_MS  60000   // search for gimbal for 1 minute after startup
AP_Mount_SToRM32::AP_Mount_SToRM32(AP_Mount &frontend, AP_Mount_Params &params, uint8_t instance) :
    AP_Mount_Backend(frontend, params, instance),
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
        case MAV_MOUNT_MODE_RETRACT: {
            const Vector3f &target = _params.retract_angles.get();
            _angle_rad.roll = radians(target.x);
            _angle_rad.pitch = radians(target.y);
            _angle_rad.yaw = radians(target.z);
            _angle_rad.yaw_is_ef = false;
            break;
        }

        // move mount to a neutral position, typically pointing forward
        case MAV_MOUNT_MODE_NEUTRAL: {
            const Vector3f &target = _params.neutral_angles.get();
            _angle_rad.roll = radians(target.x);
            _angle_rad.pitch = radians(target.y);
            _angle_rad.yaw = radians(target.z);
            _angle_rad.yaw_is_ef = false;
            break;
        }

        // point to the angles given by a mavlink message
        case MAV_MOUNT_MODE_MAVLINK_TARGETING:

            if(_query)
                {
                if(_last_mode!=MAV_MOUNT_MODE_MAVLINK_TARGETING || _force_change==true)
                    {
                    _require_change_mode=true;
                    _mount_ack_control=false;
                    _mode_change_timer=0;
                    _mount_control_timer=0;
                    _force_change=false;
                    }
                switch(mavt_target.target_type)
                    {
                        case MountTargetType::ANGLE:
                            _rate_output=false;
                            _angle_rad = mavt_target.angle_rad;
                            if(fabsf(ToDeg(_angle_rad.pitch)-_last_tilt)>0.01f || fabsf(ToDeg(_angle_rad.roll)-_last_roll)>0.01f || fabsf(ToDeg(_angle_rad.yaw) -_last_pan)>0.01f)
                                _mount_ack_control=false;
                            if(_require_change_mode==false)
                                {
                                if((AP_HAL::millis()-_mount_control_timer)>=AP_MOUNT_STORM32_COMMAND_RETRY_MS && _mount_ack_control==false)
                                    {
                                    send_do_mount_control(_angle_rad, mode);
                                    _last_tilt=ToDeg(_angle_rad.pitch);
                                    _last_roll=ToDeg(_angle_rad.roll);
                                    _last_pan=ToDeg(_angle_rad.yaw);
                                    }
                                }
                        break;
                        case MountTargetType::RATE:
                            _rate_output=true;
                            _rate_rads = mavt_target.rate_rads;
                            if(fabsf(ToDeg(_rate_rads.pitch)-_last_tilt)>0.01f || fabsf(ToDeg(_rate_rads.roll)-_last_roll)>0.01f || fabsf(ToDeg(_rate_rads.yaw) -_last_pan)>0.01f)
                                _mount_ack_control=false;
                            if(_require_change_mode==false)
                                {
                                if((AP_HAL::millis()-_mount_control_timer)>=AP_MOUNT_STORM32_COMMAND_RETRY_MS && _mount_ack_control==false)
                                    {
                                    send_do_mount_control(_rate_rads, mode);
                                    _last_tilt=ToDeg(_rate_rads.pitch);
                                    _last_roll=ToDeg(_rate_rads.roll);
                                    _last_pan=ToDeg(_rate_rads.yaw);
                                    }
                                }
                        break;
                    }

                block_send=true;
                }
                else
                {
                switch (mavt_target.target_type) {
                case MountTargetType::ANGLE:
                    _angle_rad = mavt_target.angle_rad;
                    break;
                case MountTargetType::RATE:
                    update_angle_target_from_rate(mavt_target.rate_rads, _angle_rad);
                    break;
                    }
                // do nothing because earth-frame angle targets (i.e. _angle_ef_target_rad) should have already been set by a MOUNT_CONTROL message from GCS
                resend_now = true;
                }
            break;

        // RC radio manual angle control, but with stabilization from the AHRS
        case MAV_MOUNT_MODE_RC_TARGETING: {

            if(_query)
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
                        MountTarget rc_target {};
                        if(get_rc_rate_target(rc_target)==true)
                            send_do_mount_control(rc_target,mode);
                        }
                        else
                        {
                        // update targets using pilot's RC inputs
                        MountTarget rc_target {};
                        if (get_rc_rate_target(rc_target)) {
                            update_angle_target_from_rate(rc_target, _angle_rad);
                        } else if (get_rc_angle_target(rc_target)) {
                            _angle_rad = rc_target;
                        }
                        block_send=false;
                        resend_now = true;
                        }
                    }

                }
                else
                {
                // update targets using pilot's RC inputs
                MountTarget rc_target {};
                if (get_rc_rate_target(rc_target)) {
                    update_angle_target_from_rate(rc_target, _angle_rad);
                } else if (get_rc_angle_target(rc_target)) {
                    _angle_rad = rc_target;
                }
                resend_now = true;
                }
            break;
        }

        // point mount to a GPS location
        case MAV_MOUNT_MODE_GPS_POINT:
            if(_query && _gimbal_options_mask&GIMBAL_NATIVE_GPS_POINT_MASK)
                {
                block_send=true;
                if(_last_mode!=MAV_MOUNT_MODE_GPS_POINT)
                    {
                    _mount_control_timer=0;
                    _mount_ack_control=false;
                    }
                if(!_last_roi_target.same_latlon_as(_roi_target))
                    _mount_ack_control=false;
                if((AP_HAL::millis()-_mount_control_timer)>=AP_MOUNT_STORM32_COMMAND_RETRY_MS && _mount_ack_control==false)
                    {
                    int32_t alt=_roi_target.alt,lat=_roi_target.lat,lon=_roi_target.lng;
                    if(_roi_target_set==true)
                        mavlink_msg_command_long_send(_chan,_sysid,_compid,MAV_CMD_DO_MOUNT_CONTROL,0,0,0,0,(float)alt/1E3,(float)lat/1E7,(float)lon/1E7,mode);
                        else
                        set_mode(MAV_MOUNT_MODE_RC_TARGETING);
                    _last_roi_target=_roi_target;
                    }
                }
                else
                {
                if (get_angle_target_to_roi(_angle_rad)) {
                    resend_now = true;
                    }
                }
            break;

        case MAV_MOUNT_MODE_HOME_LOCATION:
            if (get_angle_target_to_home(_angle_rad)) {
                resend_now = true;
            }
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
                    int32_t alt=_roi_target.alt,lat=_roi_target.lat,lon=_roi_target.lng;
                    if(_roi_target_set==true)
                        mavlink_msg_command_long_send(_chan,_sysid,_compid,MAV_CMD_DO_MOUNT_CONTROL,0,0,0,0,(float)alt/1E3,(float)lat/1E7,(float)lon/1E7,mode);
                        else
                        set_mode(MAV_MOUNT_MODE_RC_TARGETING);

                    }
                }
                else
                {
                if (get_angle_target_to_home(_angle_rad)) {
                    resend_now = true;
                    }
                }
            break;

        case MAV_MOUNT_MODE_SYSID_TARGET:
            if(_query && _gimbal_options_mask&GIMBAL_NATIVE_GPS_POINT_MASK)
                {
                block_send=true;
                int32_t alt=_roi_target.alt,lat=_roi_target.lat,lon=_roi_target.lng;
                if(_roi_target_set==true)
                    mavlink_msg_command_long_send(_chan,_sysid,_compid,MAV_CMD_DO_MOUNT_CONTROL,0,0,0,0,(float)alt/1E3,(float)lat/1E7,(float)lon/1E7,mode);
                    else
                    set_mode(MAV_MOUNT_MODE_RC_TARGETING);
                }
                else
                {
                if (get_angle_target_to_sysid(_angle_rad)) {
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
    //if (resend_now || ((AP_HAL::millis() - _last_send) > AP_MOUNT_STORM32_RESEND_MS)) {
    //    send_do_mount_control(_angle_rad);
    if ((resend_now || ((AP_HAL::millis() - _last_send) > AP_MOUNT_STORM32_RESEND_MS)) && block_send==false) {
        MAV_MOUNT_MODE send_mode=MAV_MOUNT_MODE_MAVLINK_TARGETING;
        if(_gimbal_options_mask&GIMBAL_MODE_AWARE_MASK)
            send_mode=get_mode();
        send_do_mount_control(_angle_rad, send_mode);
    }
}

// get attitude as a quaternion.  returns true on success
bool AP_Mount_SToRM32::get_attitude_quaternion(Quaternion& att_quat)
{
    att_quat.from_euler(_angle_rad.roll, _angle_rad.pitch, get_bf_yaw_angle(_angle_rad));
    return true;
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

    // we expect that instance 0 has compid = MAV_COMP_ID_GIMBAL, instance 1 has compid = MAV_COMP_ID_GIMBAL2, etc
    uint8_t compid = (_instance == 0) ? MAV_COMP_ID_GIMBAL : MAV_COMP_ID_GIMBAL2 + (_instance - 1);
    if (GCS_MAVLINK::find_by_mavtype_and_compid(MAV_TYPE_GIMBAL, compid, _sysid, _chan)) {
        _compid = compid;
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

    //GMB_OPTIONS_MASK only exists on comp 26, for now so hard code
    if(((AP_HAL::millis() - _last_gimbal_info_request) > AP_MOUNT_STORM32_RESEND_QUERY_MS))
        {
        mavlink_msg_param_request_read_send(_chan,_sysid,26,AP_MOUNT_GIMBAL_OPTIONS_MASK_NAME,-1);
        _last_gimbal_info_request=AP_HAL::millis();
        }
}


void AP_Mount_SToRM32::handle_param_value(const mavlink_message_t &msg)
{
    mavlink_param_value_t param_value;
    if(msg.msgid != MAVLINK_MSG_ID_PARAM_VALUE || msg.sysid!=_sysid || msg.compid!=26)
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
void AP_Mount_SToRM32::send_do_mount_control(const MountTarget& angle_target_rad,MAV_MOUNT_MODE mode)
{
    // exit immediately if not initialised
    if (!_initialised) {
        return;
    }

    // check we have space for the message
    if (!HAVE_PAYLOAD_SPACE(_chan, COMMAND_LONG)) {
        return;
    }

    // send command_long command containing a do_mount_control command
    // Note: pitch and yaw are reversed
    mavlink_msg_command_long_send(_chan,
                                  _sysid,
                                  _compid,
                                  MAV_CMD_DO_MOUNT_CONTROL,
                                  0,        // confirmation of zero means this is the first time this message has been sent
                                  -degrees(angle_target_rad.pitch),
                                  degrees(angle_target_rad.roll),
                                  -degrees(get_bf_yaw_angle(angle_target_rad)),
                                  0, 0, 0,  // param4 ~ param6 unused
                                  mode);

    // store time of send
    _last_send = AP_HAL::millis();
}

 bool AP_Mount_SToRM32::has_pan_control() const
 {

    return (_gimbal_options_mask&GIMBAL_CONTINUOUS_PAN_MASK);
 }
#endif // HAL_MOUNT_STORM32MAVLINK_ENABLED
