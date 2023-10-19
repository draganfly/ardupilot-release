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
   The Daiwa winch is produced by a Japanese company called Okaya.
   There are two PWM controls supported:
     - the rate control for releasing (high PWM) or retracting (low PWM) the line
     - the clutch control has three settings:
         - released (high PWM) lets the winch spin freely
         - engaged soft (middle PWM) allows the rate control to work but it may slip
           if too much tension is required.  This driver does not use this setting.
         - engaged hard (low PWM) allows the rate control to work regardless of tension.
   A telemetry output from the winch is connected to the autopilot and provides
   the amount of line released, tension, clutch setting, etc.
*/

#pragma once
#include <GCS_MAVLink/GCS.h>
#include <AP_Winch/AP_Winch_Backend.h>
#include <SRV_Channel/SRV_Channel.h>

#define WINCH_MAV_COMMAND_TIMEOUT 100
#define WINCH_COMMAND_REPEAT_INTERVAL 3000

class AP_Winch_Mavlink : public AP_Winch_Backend {
public:

    using AP_Winch_Backend::AP_Winch_Backend;

    // true if winch is healthy
    bool healthy() const override;

    // initialise the winch
    void init() override;

    // read telemetry from the winch and output controls
    void update() override;

    // returns current length of line deployed
    float get_current_length() const override { return Status.line_length; }

    // send status to ground station
    void send_status(const GCS_MAVLINK &channel) override;

    // write log
    void write_log() override;

    // handle mavlink status message
    void handleMessage(const mavlink_winch_status_t &msg) override;

    // handle mavlink ack message
    void handle_command_ack(const mavlink_command_ack_t &msg) override;

    // get ground sense
    int ground_sense() override;

    //get current line length
    float get_line_length() override;


private:

    // read incoming data from winch and update intermediate and latest structures
    void read_data_from_winch();

    // update pwm outputs to control winch
    void control_winch();

    uint32_t data_update_ms;            // system time that latest was last updated

    mavlink_winch_status_t Status;
    uint32_t command_send_time;         // time the last command was sent
    bool command_acked;                 // flag to indicate if command was acked

    uint8_t last_mode;

    float       last_length_desired;
    float       last_rate_desired;
};
