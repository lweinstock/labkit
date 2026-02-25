#ifndef LK_OM70L_HH
#define LK_OM70L_HH

#include <memory>
#include <vector>

#include <labkit/devices/basicdevice.hh>
#include <labkit/protocols/modbustcp.hh>
#include <labkit/comms/tcpipcomm.hh>

namespace labkit {

class Om70L : public BasicDevice {
public:
    Om70L() : BasicDevice("Baumer,OM70-L") {};
    Om70L(std::unique_ptr<TcpipComm> t_tcpip)
        { this->connect(std::move(t_tcpip)); }
    ~Om70L() {};

    /// OM70 default port 502
    static constexpr unsigned PORT = 502;

    /// LDS measurement quality
    enum Quality : unsigned {GOOD = 0, MEDIUM, BAD};

    /// Connect to OM70-L LDS via TCPIP
    void connect(std::unique_ptr<TcpipComm> t_tcpip);

    /// Turn laser diode on
    void enableLaser(bool t_ena = true);
    /// Turn laser diode off
    void disableLaser() { this->enableLaser(false); }

    /// LDS has a timeout for TCP/IP connection, default = 300 sec (WTF?!)
    void setSessionTimeout(unsigned t_timeout_sec);
    /// Returns current session timeout
    unsigned getSessionTimeout();

    /// Returns single distance measurement in [mm]
    float getMeasurement();

    /// Returns quality of last distance measurement
    Quality getQuality() { return m_quality; } 

    /// Returns sample rate of last distance measurement in [Hz]
    float getSampleRate() { return m_sr; }

    /// Returns exposure rating of last distance measurement (low=bad, high=good)
    float getExposure() { return m_exp; }

    /// Returns 100 distance measurements from memory in [mm]
    std::vector<float> getMeasurementMem();

    /// Returns quality of last 100 distance measurements
    std::vector<Quality> getQualityMem() { return m_quality_vec; }

    /// Returns sample rate of last 100 distance measurements
    std::vector<float> getSampleRateMem() { return  m_sr_vec; }

    /// Returns exposure rating of last 100 distance measurements
    std::vector<float> getExposureMem() { return m_exp_vec; }

private:
    static constexpr uint8_t UNIT_ID = 0x01;

    // Address space holding registers (FC03/06/16)
    static constexpr uint16_t ADDR_CONF_ON = 0x0000;
    static constexpr uint16_t ADDR_CONF_OFF = 0x0001;
    static constexpr uint16_t ADDR_TIMEOUT = 0x0002;
    static constexpr uint16_t ADDR_ENA_LASER = 0x019A;
    // Address space input registers (FC04)
    static constexpr uint16_t ADDR_VEND_INFO = 0x0000;
    static constexpr uint16_t ADDR_DEV_INFO = 0x0028;
    static constexpr uint16_t ADDR_ALL_MEAS = 0x00C8;
    // Block memory input registers
    static constexpr uint16_t ADDR_BLK_MEM0 = 0x0258;
    static constexpr uint16_t ADDR_BLK_MEM1 = 0x02C8;
    static constexpr uint16_t ADDR_BLK_MEM2 = 0x0338;
    static constexpr uint16_t ADDR_BLK_MEM3 = 0x03A8;
    static constexpr uint16_t ADDR_BLK_MEM4 = 0x0418;
    static constexpr uint16_t ADDR_BLK_MEM5 = 0x0488;
    static constexpr uint16_t ADDR_BLK_MEM6 = 0x04F8;
    static constexpr uint16_t ADDR_BLK_MEM7 = 0x0568;
    static constexpr uint16_t ADDR_BLK_MEM8 = 0x05D8;
    static constexpr uint16_t ADDR_BLK_MEM9 = 0x0648;
    static constexpr uint16_t ADDR_BLK_MEM10 = 0x06B8;
    static constexpr uint16_t ADDR_BLK_MEM11 = 0x0728;
    static constexpr uint16_t ADDR_BLK_MEM12 = 0x0798;
    static constexpr uint16_t ADDR_BLK_MEM13 = 0x0808;
    static constexpr uint16_t ADDR_BLK_MEM14 = 0x0878;

    /// Initialise device and setup modbus
    void init();

    /**
     * @brief Extracts the measurements from the 16x 16-bit block memory
     *
     * Warning: The format is (for some stupid reason) different to the
     * "All Measurements" format (compare manual p.60 & p.66 WTF?!?!)
     */
    void extractMemMeas(std::vector<uint16_t> t_data, float &t_dist, 
        Quality &t_quality, float &t_sample_rate, float &t_exposure);

    ModbusTcp m_modbus {};
    Quality m_quality {GOOD};
    float m_dist {0}, m_sr {0}, m_exp {0};
    std::vector<Quality> m_quality_vec {};
    std::vector<float> m_dist_vec {}, m_sr_vec {}, m_exp_vec {};
    bool m_config_mode {false};
};

}

#endif