#ifndef LK_SDG1000X_HH
#define LK_SDG1000X_HH

#include <labkit/devices/functiongenerator.hh>
#include <labkit/comms/usbtmccomm.hh>
#include <labkit/comms/tcpipcomm.hh>
#include <labkit/protocols/scpi.hh>
#include <memory>

namespace labkit
{

class Sdg1000X : public FunctionGenerator
{
/**
 * @brief Class for the Siglent SDG1000X-series function generators
 * 
 */
public:
    Sdg1000X() : FunctionGenerator(2, "Siglent,SDG1000X") {};
    Sdg1000X(std::unique_ptr<TcpipComm> t_tcpip);
    Sdg1000X(std::unique_ptr<UsbTmcComm> t_usbtmc);
    ~Sdg1000X() {};

    /* Rigol SDG1000X specific definitions */

    /// Rigol SDG1032X Vendor ID
    static constexpr uint16_t SDG1032X_VID = 0xF4EC;
    /// Rigol SDG1032X Product ID
    static constexpr uint16_t SDG1032X_PID = 0x1103;
    /// Default port of Rigol SDG1032X
    static constexpr unsigned PORT = 5025;

    /// Connect to Rigol DG4000 via TCP/IP connection
    void connect(std::unique_ptr<TcpipComm> t_tcpip);

    /// Connect to Rigol DG4000 via USBTMC
    void connect(std::unique_ptr<UsbTmcComm> t_usbtmc);

    /* Generic function generator definitions */

    /// Turn channel on/off
    void enableChannel(unsigned t_channel, bool t_enable = true) override;
    /// Returns true if channel is enabled
    bool channelEnabled(unsigned t_channel) override;

    void setWaveform(unsigned t_channel, Waveform t_wvfm) override;
    Waveform getWaveform(unsigned t_channel) override;

    /// Set signal frequency in [Hz]
    void setFrequency(unsigned t_channel, double t_freq_hz) override;
    /// Returns signal frequency in [Hz]
    double getFrequency(unsigned t_channel) override;

    /// Set signal duty cycle in [%] (0.0 - 1.0)
    void setDutyCycle(unsigned t_channel, double t_dcl) override;
    /// Returns signal duty cycle in [%] (0.0 - 1.0)
    double getDutyCycle(unsigned t_channel) override;

    /// Set signal phase in [deg] (0.0 - 360.0)
    void setPhase(unsigned t_channel, double t_phase_deg) override;
    /// Returns signal phase in [deg] (0.0 - 360.0)
    double getPhase(unsigned t_channel) override;

    /// Set signal amplitude in [V]
    void setAmplitude(unsigned t_channel, double t_ampl_v) override;
    /// Returns signal amplitude in [V]
    double getAmplitude(unsigned t_channel) override;

    /// Set signal offset in [V]
    void setOffset(unsigned t_channel, double t_offset_v) override;
    /// Returns signal offset in [V]
    double getOffset(unsigned t_channel) override;

    /// Set rising edge in [s] (10% - 90%)
    void setRisingEdge(unsigned t_channel, double t_rise_s) override;
    /// Returns rising edge in [s] (10% - 90%)
    double getRisingEdge(unsigned t_channel) override;
    /// Set falling edge in [s] (10% - 90%)
    void setFallingEdge(unsigned t_channel, double t_fall_s) override;
    /// Returns falling edge in [s] (10% - 90%)
    double getFallingEdge(unsigned t_channel) override;

    /// Set pulse width in [s]
    void setPulseWidth(unsigned t_channel, double t_width_s) override;
    /// Returns pulse width in [s]
    double getPulseWidth(unsigned t_channel) override;

private:
    void init();

    /// Converts measurement to Rigol SDG1000X SCPI compatible string
    static std::string wvfmToString(Waveform t_wfvm);

    /**
     * @brief Returns the specified parameter from BaSic WaVeform string (BSWV)
     * 
     * @param t_bswv BaSic WaVeform (BSWV) string
     * @param t_par Parameter (e.g. OFST, AMP, PHSE, ...)
     * @return Returns a string containing the parameter value
     */
    std::string getBswvVal(const std::string &t_bswv, const std::string &t_par);    

    Scpi m_scpi{};
};

}

#endif