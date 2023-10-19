#include "Copter.h"

#if MODE_BRAKE_ENABLED == ENABLED

/*
 * Init and run calls for brake flight mode
 */

static uint32_t update_time_ms;

// brake_init - initialise brake controller
bool ModeBrake::init(bool ignore_checks)
{
    update_time_ms=0;
    // initialise pos controller speed and acceleration
    pos_control->set_max_speed_accel_xy(inertial_nav.get_velocity_neu_cms().length(), BRAKE_MODE_DECEL_RATE);
    pos_control->set_correction_speed_accel_xy(inertial_nav.get_velocity_neu_cms().length(), BRAKE_MODE_DECEL_RATE);

    // initialise position controller
    pos_control->init_xy_controller();

    // set vertical speed and acceleration limits
    pos_control->set_max_speed_accel_z(BRAKE_MODE_SPEED_Z, BRAKE_MODE_SPEED_Z, BRAKE_MODE_DECEL_RATE);
    pos_control->set_correction_speed_accel_z(BRAKE_MODE_SPEED_Z, BRAKE_MODE_SPEED_Z, BRAKE_MODE_DECEL_RATE);

    // initialise the vertical position controller
    if (!pos_control->is_active_z()) {
        pos_control->init_z_controller();
    }

    _timeout_ms = 0;

    return true;
}

// brake_run - runs the brake controller
// should be called at 100hz or more
void ModeBrake::run()
{
    // if not armed set throttle to zero and exit immediately
    if (is_disarmed_or_landed()) {
        make_safe_ground_handling();
        pos_control->relax_z_controller(0.0f);
        return;
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // relax stop target if we might be landed
    if (copter.ap.land_complete_maybe) {
        pos_control->soften_for_landing_xy();
    }

    // use position controller to stop
    Vector2f vel;
    Vector2f accel;
    pos_control->input_vel_accel_xy(vel, accel);
    pos_control->update_xy_controller();

    // call attitude controller
    attitude_control->input_thrust_vector_rate_heading(pos_control->get_thrust_vector(), 0.0f);

    pos_control->set_pos_target_z_from_climb_rate_cm(0.0f);
    pos_control->update_z_controller();

    if (_timeout_ms != 0 && millis()-_timeout_start >= _timeout_ms) {
        if (!copter.set_mode(Mode::Number::LOITER, ModeReason::BRAKE_TIMEOUT)) {
            copter.set_mode(Mode::Number::ALT_HOLD, ModeReason::BRAKE_TIMEOUT);
        }
    }
}

void ModeBrake::timeout_to_loiter_ms(uint32_t timeout_ms)
{
    _timeout_start = millis();
    _timeout_ms = timeout_ms;
}

void ModeBrake::set_yaw_target(bool use_yaw, float yaw_cd, bool use_yaw_rate, float yaw_rate_cds, bool relative_angle)
{
    update_time_ms=millis();
    if (use_yaw && relative_angle) {
        auto_yaw.set_fixed_yaw(yaw_cd * 0.01f, 0.0f, 0, relative_angle);
    } else if (use_yaw && use_yaw_rate) {
        auto_yaw.set_yaw_angle_rate(yaw_cd * 0.01f, yaw_rate_cds * 0.01f);
    } else if (use_yaw && !use_yaw_rate) {
        auto_yaw.set_yaw_angle_rate(yaw_cd * 0.01f, 0.0f);
    } else if (use_yaw_rate) {
        auto_yaw.set_rate(yaw_rate_cds);
    } else {
        auto_yaw.set_mode_to_default(false);
    }
}

// return guided mode timeout in milliseconds. Only used for velocity, acceleration, angle control, and angular rates
uint32_t ModeBrake::get_timeout_ms() const
{
    return MAX(copter.g2.guided_timeout, 0.1) * 1000;
}

#endif
