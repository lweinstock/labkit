#ifndef LK_FUNCTIONGENERATOR_HH
#define LK_FUNCTIONGENERATOR_HH

#include <labkit/devices/basicdevice.hh>

namespace labkit
{

/**
 * @brief Abstract base class for function generator
 * 
 *  This base class defines a minimal set of functions a device has to provide
 *  to be an function generator. 
 *
 *  This abstraction allows to swap function generators in a setup if only 
 *  basic, non-device-specific functionality (e.g. sending a sine) is required.
 */
class FunctionGenerator : public BasicDevice
{
public:
    virtual ~FunctionGenerator() {};

    /// Turn channel on/off
    virtual void enableChannel(unsigned t_channel, bool t_enable = true) = 0;
    /// Turn channel off
    void disableChannel(unsigned t_channel) { enableChannel(t_channel, false); }
    /// Returns true if channel is enabled
    virtual bool channelEnabled(unsigned t_channel) = 0;

    /// Waveform definitions
    enum Waveform : unsigned {SINE = 0, SQUARE, RAMP, PULSE, NOISE, DC};
    virtual void setWaveform(unsigned t_channel, Waveform t_wvfm) = 0;
    virtual Waveform getWaveform(unsigned t_channel) = 0;

    /// Set signal frequency in [Hz]
    virtual void setFrequency(unsigned t_channel, double t_freq_hz) = 0;
    /// Returns signal frequency in [Hz]
    virtual double getFrequency(unsigned t_channel) = 0;

    /// Set signal duty cycle in [%] (0.0 - 1.0)
    virtual void setDutyCycle(unsigned t_channel, double t_dcl) = 0;
    /// Returns signal duty cycle in [%] (0.0 - 1.0)
    virtual double getDutyCycle(unsigned t_channel) = 0;

    /// Set signal phase in [deg] (0.0 - 360.0)
    virtual void setPhase(unsigned t_channel, double t_phase_deg) = 0;
    /// Returns signal phase in [deg] (0.0 - 360.0)
    virtual double getPhase(unsigned t_channel) = 0;

    /// Set signal amplitude in [V]
    virtual void setAmplitude(unsigned t_channel, double t_ampl_v) = 0;
    /// Returns signal amplitude in [V]
    virtual double getAmplitude(unsigned t_channel) = 0;

    /// Set signal offset in [V]
    virtual void setOffset(unsigned t_channel, double t_offset_v) = 0;
    /// Returns signal offset in [V]
    virtual double getOffset(unsigned t_channel) = 0;

    /// Set rising edge in [s] (10% - 90%)
    virtual void setRisingEdge(unsigned t_channel, double t_rise_s) = 0;
    /// Returns rising edge in [s] (10% - 90%)
    virtual double getRisingEdge(unsigned t_channel) = 0;
    /// Set falling edge in [s] (10% - 90%)
    virtual void setFallingEdge(unsigned t_channel, double t_fall_s) = 0;
    /// Returns falling edge in [s] (10% - 90%)
    virtual double getFallingEdge(unsigned t_channel) = 0;

    /// Set pulse width in [s]
    virtual void setPulseWidth(unsigned t_channel, double t_width_s) = 0;
    /// Returns pulse width in [s]
    virtual double getPulseWidth(unsigned t_channel) = 0;

protected:
    FunctionGenerator() : BasicDevice("Unknown function generator") {};

    FunctionGenerator(unsigned t_channels) 
      : BasicDevice("Unknown function generator", t_channels) {};

    FunctionGenerator(std::string t_name, unsigned t_channels) 
      : BasicDevice(t_name, t_channels) {};
};

}

#endif