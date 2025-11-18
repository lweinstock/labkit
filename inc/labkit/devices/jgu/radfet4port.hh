#ifndef LK_RADFET_QUAD_HH
#define LK_RADFET_QUAD_HH

#include <labkit/devices/basicdevice.hh>
#include <labkit/comms/tcpipcomm.hh>

namespace labkit 
{
/**
 * @brief 4-channel readout for Varadis RadFETs developed by KPH at the JGU
 * 
 */
class RadFET4Port : public labkit::BasicDevice
{
public:
    RadFET4Port() : BasicDevice("RADFet4Port") {};
    RadFET4Port(std::unique_ptr<TcpipComm> t_tcpip);
    ~RadFET4Port() {};

    /// Default port
    static constexpr unsigned PORT = 5000;
    
    /// Connect to RadFET readout via TCP/IP connection 
    void connect(std::unique_ptr<TcpipComm> t_tcpip);

    /**
     * @brief Read RadFET threshold voltage in [V]
     * 
     * @param[in] t_channel RadFET module channel (0-3)
     * @param[out] t_raw_adc Raw ADC counts
     * @return float Threshold voltage in [V]
     */
    float getFETVoltage(unsigned t_channel, uint32_t &t_raw_adc);
    /// Read RadFET threshold voltage in [V]
    float getFETVoltage(unsigned t_channel);

    /**
     * @brief Read module temperatur diode voltage in [V]
     * 
     * @param[in] t_channel RadFET module channel (0-3)
     * @param[out] t_raw_adc Raw ADC counts
     * @return float Temperature diode forward voltage in [V]
     */
    float getDiodeVoltage(unsigned t_channel, uint32_t &t_raw_adc);
    /// Read module temperatur diode voltage in [V]
    float getDiodeVoltage(unsigned t_channel);

    /// Reboot the on-board microcontroller
    void reboot();

private:
    void init();
};

}

#endif