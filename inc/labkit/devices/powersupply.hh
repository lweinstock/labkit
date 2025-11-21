#ifndef LK_POWER_SUPPLY_HH
#define LK_POWER_SUPPLY_HH

#include <labkit/devices/basicdevice.hh>

namespace labkit
{

/**
 * @brief Abstract base class for lab power supplies
 * 
 *  This base class defines a minimal set of functions a device has to provide
 *  to be an power supply. 
 *
 *  This abstraction allows to swap power supplies in a setup if only basic,
 *  non-device-specific functionality (e.g. setting voltages) is required.
 */
class PowerSupply : public BasicDevice
{
public:
    virtual ~PowerSupply() {};

    /// Turn channel on/off
    virtual void enableChannel(unsigned t_channel, bool t_enable = true) = 0;
    /// Turn channel off
    void disableChannel(unsigned t_channel) { enableChannel(t_channel, false); }
    /// Returns true if channel is enabled
    virtual bool channelEnabled(unsigned t_channel) = 0;

    /// Set output voltage in [V]
    virtual void setVoltage(unsigned t_channel, double t_volts) = 0;
    /// Returns set voltage in [V]
    virtual double getVoltage(unsigned t_channel) = 0;
    /// Returns current output voltage in [V]
    virtual double measureVoltage(unsigned t_channel) = 0;

    /// Set maximum output current in [A]
    virtual void setCurrent(unsigned t_channel, double t_volts) = 0;
    /// Returns maximum output current in [A]
    virtual double getCurrent(unsigned t_channel) = 0;
    /// Returns current output current in [A]
    virtual double measureCurrent(unsigned t_channel) = 0;

protected:
    PowerSupply() : BasicDevice("Unknown power supply") {};

    PowerSupply(unsigned t_channels) 
      : BasicDevice("Unknown power supply", t_channels) {};

    PowerSupply(std::string t_name, unsigned t_channels) 
      : BasicDevice(t_name, t_channels) {};
};

}

#endif