#ifndef LK_HMP4000_HH
#define LK_HMP4000_HH

#include <labkit/devices/powersupply.hh>
#include <labkit/comms/tcpipcomm.hh>
#include <labkit/comms/serialcomm.hh>
#include <labkit/comms/usbtmccomm.hh>
#include <labkit/protocols/scpi.hh>

namespace labkit 
{

/**
 * @brief Class for the Rohde & Schwarz HMP4000-series lab power supplies
 * 
 */
class Hmp4000 : public PowerSupply
{
public:
    Hmp4000() : PowerSupply("Rohde&Schwarz,HMP4000", 4) {};
    Hmp4000(std::unique_ptr<TcpipComm> t_tcpip);
    Hmp4000(std::unique_ptr<SerialComm> t_serial);
    Hmp4000(std::unique_ptr<UsbTmcComm> t_usbtmc);
    ~Hmp4000() {};

    /* Rohde & Schwarz HMP4000 specific definitions */

    /// Default port of Rohde & Schwarz HMP4000
    static constexpr unsigned PORT = 5025;
    /// Rohde & Schwarz HMP4000 Vendor ID
    static constexpr uint16_t HMP4040_VID = 0x0AAD;
    /// Rohde & Schwarz HMP4000 Product ID
    static constexpr uint16_t HMP4040_PID = 0x0117;

    /// Connect to Rohde & Schwarz HMP4000 via TCP/IP connection
    void connect(std::unique_ptr<TcpipComm> t_tcpip);

    /// Connect to Rohde & Schwarz HMP4000 via serial connection
    void connect(std::unique_ptr<SerialComm> t_serial);

    /// Connect to Rohde & Schwarz HMP4000 via USBTMC
    void connect(std::unique_ptr<UsbTmcComm> t_usbtmc);

    /* Generic power supply definitions */

    /// Turn channel on/off
    void enableChannel(unsigned t_channel, bool t_enable = true) override;
    /// Returns true if channel is enabled
    bool channelEnabled(unsigned t_channel) override;

    /// Set output voltage in [V] (HMP range = 0 - 32.05V)
    void setVoltage(unsigned t_channel, double t_volts) override;
    /// Returns set voltage in [V]
    double getVoltage(unsigned t_channel) override;
    /// Returns current output voltage in [V]
    double measureVoltage(unsigned t_channel) override;

    /// Set maximum output current in [A]
    void setCurrent(unsigned t_channel, double t_amps) override;
    /// Returns maximum output current in [A]
    double getCurrent(unsigned t_channel) override;
    /// Returns current output current in [A]
    double measureCurrent(unsigned t_channel) override;

private:
    void init();

    /**
     * @brief Switch current channel of HMP4000 to change settings
     * 
     * The HMP4000 implements different channels as different SCPI instruments
     * and setting changes are always applied to the current instrument.
     *
     * @param t_channel Output channel
     */
    void selectChannel(unsigned t_channel);

    /// Hameg is quite slow to process queries, therefore a query delay is added
    static constexpr unsigned QUERY_DELAY_MS = 10;

    Scpi m_scpi{};
};

}

#endif