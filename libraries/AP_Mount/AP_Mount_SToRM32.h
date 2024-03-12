/*
  SToRM32 mount backend class
 */
#pragma once

#include "AP_Mount_Backend.h"

#define AP_MOUNT_STORM32_QUERY_GIMBAL_MS 20000  //try for up to 20 seconds after finding a gimbal to get a GIMBAL_DEVICE_INFORMATION message
#define AP_MOUNT_STORM32_RESEND_QUERY_MS 1000
#define AP_MOUNT_STORM32_RESEND_MODE_MS 5000
#define AP_MOUNT_STORM32_COMMAND_RETRY_MS 50

#define AP_MOUNT_GIMBAL_OPTIONS_MASK_NAME "GMB_OPTIONS_MASK"

#if HAL_MOUNT_STORM32MAVLINK_ENABLED

#include <AP_Math/AP_Math.h>
#include <AP_Common/AP_Common.h>

class AP_Mount_SToRM32 : public AP_Mount_Backend
{
#define GIMBAL_MODE_AWARE_MASK (1<<0)
#define GIMBAL_PREFER_RATE_CONTROL_MASK (1<<1)
#define GIMBAL_NATIVE_GPS_POINT_MASK (1<<2)
#define GIMBAL_CONTINUOUS_PAN_MASK (1<<3)
public:
    // Constructor
    AP_Mount_SToRM32(AP_Mount &frontend, AP_Mount_Params &params, uint8_t instance);

    // init - performs any required initialisation for this instance
    void init() override {}

    // update mount position - should be called periodically
    void update() override;

    // has_pan_control - returns true if this mount can control its pan (required for multicopters)
    bool has_pan_control() const override;// { return yaw_range_valid(); }

    void handle_param_value(const mavlink_message_t &msg) override;
    
    void handle_command_ack(const mavlink_message_t &msg) override;

protected:

    // get attitude as a quaternion.  returns true on success
    bool get_attitude_quaternion(Quaternion& att_quat) override;

private:

    // search for gimbal in GCS_MAVLink routing table
    void find_gimbal();

    // request the GIMBAL_DEVICE_INFORMATION message
    void query_gimbal();

    // send_do_mount_control with latest angle targets
    void send_do_mount_control(const MountTarget& angle_target_rad,MAV_MOUNT_MODE mode);

    // internal variables
    bool _initialised;              // true once the driver has been initialised
    bool _query;
    uint8_t _sysid;                 // sysid of gimbal
    uint8_t _compid;                // component id of gimbal
    mavlink_channel_t _chan = MAVLINK_COMM_0;        // mavlink channel used to communicate with gimbal
    uint32_t _last_send;            // system time of last do_mount_control sent to gimbal
    MountTarget _angle_rad;         // latest angle target
    MountTarget _rate_rads;         //latest rate target

    uint32_t _last_gimbal_info_request; //system time of the last time we send the request for the gimbal info
    uint32_t _gimbal_options_mask;
    MAV_MOUNT_MODE _last_mode;
    bool _require_change_mode;
    bool _rate_output;
    uint32_t _mode_change_timer;
    bool _mount_ack_control=false;
    uint32_t _mount_control_timer;
    bool _force_change;

    struct Location _last_roi_target;
    float _last_tilt,_last_roll,_last_pan;
};
#endif // HAL_MOUNT_STORM32MAVLINK_ENABLED
