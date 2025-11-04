#ifndef LK_DS1000Z_HH
#define LK_DS1000Z_HH

#include <labkit/devices/oscilloscope.hh>
#include <labkit/comms/usbtmccomm.hh>
#include <labkit/comms/tcpipcomm.hh>
#include <labkit/protocols/scpi.hh>

#include <memory>

namespace labkit
{
/**
 * @brief Class for the Rigol DS1000Z-series oscilloscopes
 * 
 */
class Ds1000Z : public Oscilloscope
{
public:
    Ds1000Z() : Oscilloscope(4, "Rigol,DS1000Z") {};
    Ds1000Z(std::unique_ptr<TcpipComm> t_tcpip); 
    Ds1000Z(std::unique_ptr<UsbTmcComm> t_usbtmc); 
    ~Ds1000Z() { m_scpi.clearComm(); };

    /* Rigol DS1000Z specific definitions */

    /// Rigol DS1104 Vendor ID
    static constexpr uint16_t DS1104_VID = 0x1AB1;
    /// Rigol DS1104 Product ID
    static constexpr uint16_t DS1104_PID = 0x04CE;
    /// Default port of Rigol DS1000Z
    static constexpr unsigned PORT = 5555;

    /// Connect to Rigol DS1000Z via TCP/IP connection
    void connect(std::unique_ptr<TcpipComm> t_tcpip);

    /// Connect to Rigol DS1000Z via USBTMC
    void connect(std::unique_ptr<UsbTmcComm> t_usbtmc);

    virtual void disconnect() override;

    /* Generic oscilloscope definitions */

    /// Turn channel on/off
    void enableChannel(unsigned t_channel, bool t_enable = true) override;
    /// Returns true if channel is enabled
    bool channelEnabled(unsigned t_channel) override;

    /// Set attenuation
    void setAtten(unsigned t_channel, double t_att) override;
    /// Returns attenuation
    double getAtten(unsigned t_channel) override;

    /// Set vertical base in [V/div]
    void setVertBase(unsigned t_channel, double t_volts_per_div) override;
    /// Returns vertical base in [V/div]
    double getVertBase(unsigned t_channel) override;
    /// Set vertical offset in [V]
    void setVertOffset(unsigned t_channel, double t_offset_v) override;
    /// Returns vertical offset in [V]
    double getVertOffset(unsigned t_channel) override;

    /// Set horizontal base in [V/div]
    void setHorzBase(double t_sec_per_div) override;
    /// Returns horizontal base in [V/div]
    double getHorzBase() override;
    /// Set horizontal offset in [V]
    void setHorzOffset(double t_offset_s) override;
    /// Returns horizontal offset in [V]
    double getHorzOffset() override;

    /// Set single channel measurement
    void setMeasurement(unsigned t_channel, MeasurementItem t_meas) override;
    /// Get result of single channel measurement
    double getMeasurement(unsigned t_channel, MeasurementItem t_meas) override;
    /// Set dual channel measurement
    void setMeasurement(unsigned t_channel1, unsigned t_channel2, 
        MeasurementItem t_meas) override;
    /// Get result of dual channel measurement
    double getMeasurement(unsigned t_channel1, unsigned t_channel2, 
        MeasurementItem t_meas) override;
    /// Clear all measurements
    void clearMeasurements() override;

    /// Run acquisition
    void run() override;
    /// Stop acquisition
    void stop() override;
    /// Single acquisition
    void singleShot() override;

    void setTriggerType(TriggerType t_trig) override;
    void setTriggerLevel(double t_level) override;
    void setTriggerSource(unsigned t_channel) override;

    /// Returns true if trigger conditions have been met
    bool triggered() override;
    /// Returns true if data acquisition has stopped
    bool stopped() override;

    /// Read sample data
    void readSampleData(unsigned t_channel, std::vector<double> &t_horz_data, 
        std::vector<double> &t_vert_data) override;

private:
    void init();
    void setMemoryDataRange(unsigned t_sta, unsigned t_sto);
    std::vector<uint8_t> readMemoryData();

    /// Converts measurement to Rigol DS1000Z SCPI compatible string
    static std::string measToString(MeasurementItem t_meas);
    /// Converts trigger type to Rigol DS1000Z SCPI compatible string
    static std::string trigToString(TriggerType t_trig);

    Scpi m_scpi {};

};

}

#endif