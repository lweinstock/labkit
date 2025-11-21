#ifndef LK_OSCILLOSCOPE_HH
#define LK_OSCILLOSCOPE_HH

#include <labkit/devices/basicdevice.hh>

namespace labkit
{

/**
 * @brief Abstract base class for oscilloscopes
 * 
 *  This base class defines a minimal set of functions a device has to provide
 *  to be an oscilloscope. 
 *
 *  This abstraction allows to swap oscilloscopes in a setup if only basic,
 *  non-device-specific functionality (e.g. recording a transient) is required.
 */
class Oscilloscope : public BasicDevice
{
public:
    virtual ~Oscilloscope() {};

    /// Turn channel on/off
    virtual void enableChannel(unsigned t_channel, bool t_enable = true) = 0;
    /// Turn channel off
    void disableChannel(unsigned t_channel) { enableChannel(t_channel, false); }
    /// Returns true if channel is enabled
    virtual bool channelEnabled(unsigned t_channel) = 0;

    /// Set attenuation
    virtual void setAtten(unsigned t_channel, double t_att) = 0;
    /// Returns attenuation
    virtual double getAtten(unsigned t_channel) = 0;

    /// Set vertical base in [V/div]
    virtual void setVertBase(unsigned t_channel, double t_volts_per_div) = 0;
    /// Returns vertical base in [V/div]
    virtual double getVertBase(unsigned t_channel) = 0;
    /// Set vertical offset in [V]
    virtual void setVertOffset(unsigned t_channel, double t_offset_v) = 0;
    /// Returns vertical offset in [V]
    virtual double getVertOffset(unsigned t_channel) = 0;

    /// Set horizontal base in [s/div]
    virtual void setHorzBase(double t_sec_per_div) = 0;
    /// Returns horizontal base in [s/div]
    virtual double getHorzBase() = 0;
    /// Set horizontal offset in [s]
    virtual void setHorzOffset(double t_offset_s) = 0;
    /// Returns horizontal offset in [s]
    virtual double getHorzOffset() = 0;

    /// Single and dual channel measurements
    enum MeasurementItem : unsigned 
    {
        VMAX = 0,   //< Maximum voltage
        VMIN,       //< Minimum voltage
        VPP,        //< Peak-to-peak voltage
        VTOP,       //< Top voltage
        VBASE,      //< Base voltage
        VAMP,       //< Voltage amplitude
        VAVG,       //< Average voltage
        VRMS,       //< Root-mean-square voltage
        OVERSHOOT,  //< Voltage overshoot (over top)
        PRESHOOT,   //< Voltage preshoot (under base)
        FREQ,       //< Frequency measurement
        RISETIME,   //< Signal rise time (10% - 90%)
        FALLTIME,   //< Signal fall time (90% - 10%)
        POS_WIDTH,  //< Positive signal width
        NEG_WIDTH,  //< Negative signal width
        POS_DUTY,   //< Positive duty cycle
        NEG_DUTY,   //< Negative duty cycle
        POS_DELAY,  //< Delay of positive signal
        NEG_DELAY,  //< Delay of negative signal
        POS_PHASE,  //< Phase shift of positive signal
        NEG_PHASE   //< Phase shift of negative signal
    };
    /// Set single channel measurement
    virtual void setMeasurement(unsigned t_channel, MeasurementItem t_meas) = 0;
    /// Get result of single channel measurement
    virtual double getMeasurement(unsigned t_channel, MeasurementItem t_meas) = 0;
    /// Set dual channel measurement
    virtual void setMeasurement(unsigned t_channel1, unsigned t_channel2, 
        MeasurementItem t_meas) = 0;
    /// Get result of dual channel measurement
    virtual double getMeasurement(unsigned t_channel1, unsigned t_channel2, 
        MeasurementItem t_meas) = 0;
    /// Clear all measurements
    virtual void clearMeasurements() = 0;

    /// Run acquisition
    virtual void run() = 0;
    /// Stop acquisition
    virtual void stop() = 0;
    /// Single acquisition
    virtual void singleShot() = 0;

    enum TriggerType : unsigned {RISE = 0, FALL, BOTH};
    virtual void setTriggerType(TriggerType t_trig) = 0;
    virtual void setTriggerLevel(double t_level) = 0;
    virtual void setTriggerSource(unsigned t_channel) = 0;

    /// Returns true if trigger conditions have been met
    virtual bool triggered() = 0;
    /// Returns true if data acquisition has stopped
    virtual bool stopped() = 0;

    /// Read sample data
    virtual void readSampleData(unsigned t_channel, 
        std::vector<double> &t_horz_data, std::vector<double> &t_vert_data) = 0;

protected:
    Oscilloscope() : BasicDevice("Unknown oscilloscope") {};

    Oscilloscope(unsigned t_channels) 
      : BasicDevice("Unknown oscilloscope", t_channels) {};

    Oscilloscope(std::string t_name, unsigned t_channels) 
      : BasicDevice(t_name, t_channels) {};
};

}

#endif