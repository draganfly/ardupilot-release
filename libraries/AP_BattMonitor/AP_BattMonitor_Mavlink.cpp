
#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include "AP_BattMonitor.h"
#include "AP_BattMonitor_Mavlink.h"


/// Constructor
AP_BattMonitor_Mavlink::AP_BattMonitor_Mavlink(AP_BattMonitor &mon,
                                             AP_BattMonitor::BattMonitor_State &mon_state,
                                             AP_BattMonitor_Params &params) :
    AP_BattMonitor_Backend(mon, mon_state, params)
{
    // always healthy
    CountedConsumedmAh=0;
    HaveCurrent=false;
    HavemAh=false;
    HaveWh=false;
    HavePercent=false;
    HavePercent=false;
    _state.healthy = true;
}

// read - read the voltage and current
void
AP_BattMonitor_Mavlink::read()
{
    //mutex???
    WITH_SEMAPHORE(_msem);
    uint64_t DeltaT;
    uint64_t tnow=AP_HAL::micros();
    DeltaT=tnow-_state.last_time_micros;
    if(DeltaT>5000000)  //no message for 5 seconds
        _state.healthy=false;
        else
        _state.healthy=true;
    
    _state.voltage=Voltage;
    _state.current_amps=Current;
    if(HavemAh)
        {
        _state.consumed_mah=BatteryConsumedmAh;
        }
        else
        {
        _state.consumed_mah=CountedConsumedmAh;
        }

    _state.consumed_wh=ConsumedWh;
    _state.percent=Percent;
    _state.time_remaining=TimeRemaining;
}

bool AP_BattMonitor_Mavlink::has_consumed_energy() const
{
    //mutex?
    //WITH_SEMAPHORE(_msem);
    return HaveWh;
}

bool AP_BattMonitor_Mavlink::has_current() const
{
    //mutex?
    //WITH_SEMAPHORE(_msem);
    return HaveCurrent;
}

bool AP_BattMonitor_Mavlink::has_percent() const
{
    //mutex?
    //WITH_SEMAPHORE(_msem);
    return HavePercent;
}

void AP_BattMonitor_Mavlink::handle_mavlink(const mavlink_message_t &msg, uint8_t instance )
{
    if(_params._comp_id!=msg.compid || _params._sys_id != msg.sysid || msg.msgid != MAVLINK_MSG_ID_BATTERY_STATUS)
        return;
    mavlink_battery_status_t status;
    mavlink_msg_battery_status_decode(&msg,&status);
    if(status.id!=instance)
        return;
    int i=0;
    float ftemp=0;
    while(i<10 && status.voltages[i]!=UINT16_MAX)
    {
        ftemp += ((float)status.voltages[i])/1000;
        i++;
    }
    //mutex on these????
    WITH_SEMAPHORE(_msem);
    uint64_t tNow=AP_HAL::micros();
    uint64_t Deltat=tNow-_state.last_time_micros;
    Voltage=ftemp;
    Current=((float)status.current_battery)/100;
    
    if(status.current_battery!=-1)
        {
            HaveCurrent=true;
            ftemp=Deltat/1000000; //time in seconds
            ftemp /=3600;   //time in hours
            CountedConsumedmAh+=Current*1000.0f*ftemp;
        }
        else
        {
            HaveCurrent=false;
        }
    BatteryConsumedmAh=(float)status.current_consumed;
    if(status.current_consumed!=-1)
        HavemAh=true;
        else
        HavemAh=false;
    ConsumedWh=((float)status.energy_consumed)/36;
    if(status.energy_consumed!=-1)
        HaveWh=true;
        else
        HaveWh=false;
    _state.last_time_micros=tNow;
    Percent=status.battery_remaining;
    if(Percent<0 || Percent>100)
        HavePercent=false;
        else
        HavePercent=true;
    
    TimeRemaining=status.time_remaining;
    if(TimeRemaining<=0)
        _state.has_time_remaining=false;
        else
        _state.has_time_remaining=true;
}
