#pragma once

#include "AP_BattMonitor.h"
#include "AP_BattMonitor_Backend.h"


class AP_BattMonitor_Mavlink : public AP_BattMonitor_Backend
{
public:

    /// Constructor
    AP_BattMonitor_Mavlink(AP_BattMonitor &mon, AP_BattMonitor::BattMonitor_State &mon_state, AP_BattMonitor_Params &params);

    /// Read the battery voltage and current.  Should be called at 10hz
    void read() override;

    /// returns true if battery monitor provides consumed energy info
    bool has_consumed_energy() const override;

    /// returns true if battery monitor provides current info
    bool has_current() const override;

    bool has_percent() const override;
    bool has_cell_voltages() const override;

    void init(void) override {}

    void handle_mavlink(const mavlink_message_t &msg, uint8_t instance) override;

    HAL_Semaphore _msem;

protected:
    float Voltage;
    float Current;
    float BatteryConsumedmAh;
    float CountedConsumedmAh;
    float ConsumedWh;
    bool HaveCurrent;
    bool HavemAh;
    bool HaveWh;
    bool HavePercent;
    int8_t Percent;
    bool HaveTimeRemaining;
    uint16_t TimeRemaining;
    bool HasCellVoltages;


};
