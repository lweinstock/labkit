#include <labkit/devices/baumer/om70l.hh>
#include <labkit/exceptions.hh>
#include <labkit/debug.hh>

#include <cmath>
#include <memory>

using namespace std;

namespace labkit {

void Om70L::connect(unique_ptr<TcpipComm> t_tcpip)
{
    this->setComm(std::move(t_tcpip));
    this->init();
    return;
}

void Om70L::enableLaser(bool ena)
{
    if (!m_config_mode) {
        m_modbus.writeSingleHoldingReg(UNIT_ID, ADDR_CONF_ON, 0x0001);
        m_config_mode = true;
    }

    if (ena)
        m_modbus.writeSingleHoldingReg(UNIT_ID, ADDR_ENA_LASER, 0x0001);
    else
        m_modbus.writeSingleHoldingReg(UNIT_ID, ADDR_ENA_LASER, 0x0000);

    return;
}

void Om70L::setSessionTimeout(unsigned timeout_sec)
{
    if (!m_config_mode) {
        m_modbus.writeSingleHoldingReg(UNIT_ID, ADDR_CONF_ON, 0x0001);
        m_config_mode = true;
    }
    vector<uint16_t> data{};
    data.push_back( static_cast<uint16_t>(timeout_sec & 0xFFFF) );
    data.push_back( static_cast<uint16_t>( (timeout_sec >> 16) & 0xFFFF) );
    m_modbus.writeMultipleHoldingRegs(UNIT_ID, ADDR_TIMEOUT, data);
    return;
}

unsigned Om70L::getSessionTimeout()
{
    auto resp = m_modbus.readMultipleHoldingRegs(UNIT_ID, ADDR_TIMEOUT, 2);
    return static_cast<unsigned>( resp.at(0) | (resp.at(1) << 16) );
}

float Om70L::getMeasurement()
{
    // Turn off config mode to increase sample rate
    if (m_config_mode) {
        m_modbus.writeSingleHoldingReg(UNIT_ID, ADDR_CONF_OFF, 0x0001);
        m_config_mode = false;
    }

    vector<uint16_t> resp{};
    resp = m_modbus.readInputRegs(UNIT_ID, ADDR_ALL_MEAS, 17);
    
    m_quality = static_cast<Quality>(resp.at(1));
    // memcopy is required, casting results in wrong conversion to integer and
    // attaches a comma (bitwise conversion vs. cast)
    uint32_t val {0x0000};
    val = resp.at(3) | (resp.at(4) << 16);
    memcpy(&m_dist, &val, sizeof(float));
    val = resp.at(5) | (resp.at(6) << 16);
    memcpy(&m_sr, &val, sizeof(float));
    val = resp.at(7) | (resp.at(8) << 16);
    memcpy(&m_exp, &val, sizeof(float));

    if (isnan(m_dist)) m_dist = -1.;

    return m_dist;
}

vector<float> Om70L::getMeasurementMem()
{
    // Turn off config mode to increase sample rate
    if (m_config_mode) {
        m_modbus.writeSingleHoldingReg(UNIT_ID, ADDR_CONF_OFF, 0x0001);
        m_config_mode = false;
    }

    // Clear old data
    m_dist_vec.clear();
    m_quality_vec.clear();
    m_sr_vec.clear();
    m_exp_vec.clear();

    vector<uint16_t> resp{};
    float dist, sample_rate, exposure;
    Quality q;
    for (unsigned i = 0; i < 15; i++) {
        resp.clear();
        // All memory blocks contain 7 measurements (7x16 = 112 registers) 
        // except the last one which contains only 2 readings (2x16 = 32 regs)
        if (i < 14)
            resp = m_modbus.readInputRegs(UNIT_ID, ADDR_BLK_MEM0+112*i, 112);
        else
            resp = m_modbus.readInputRegs(UNIT_ID, ADDR_BLK_MEM14, 32);

        // Cut the (usually) 112 registers into 7 slices of 16 registers
        unsigned n_measurements = resp.size()/16;
        for (unsigned j = 0; j < n_measurements; j++) {
            vector<uint16_t> slice(resp.begin() + 16*j, resp.begin() + 16*(j+1));
            this->extractMemMeas(slice, dist, q, sample_rate, exposure);
            m_dist_vec.push_back(dist);
            m_quality_vec.push_back(q);
            m_sr_vec.push_back(sample_rate);
            m_exp_vec.push_back(exposure);
        }
    }

    return m_dist_vec;
}

/*
 *      P R I V A T E   M E T H O D S
 */

void Om70L::init()
{
    m_modbus.setComm(this->getComm());
    this->enableLaser();
    return;
}

void Om70L::extractMemMeas(vector<uint16_t> data, float &dist, Quality &quality, 
    float &sample_rate, float &exposure)
{
    if (data.size() != 16) {
        fprintf(stderr, "OM70-L measurements are constructed from 16x"
        "16-bit registers, received %zux 16-bit\n", data.size());
        abort();
    }

    quality = static_cast<Quality>(data.at(0));
    // memcopy is required, casting results in wrong conversion to integer and
    // attaches a comma (bitwise conversion vs. cast)
    uint32_t val {0x0000};
    val = data.at(2) | (data.at(3) << 16);
    memcpy(&dist, &val, sizeof(float));
    val = data.at(4) | (data.at(5) << 16);
    memcpy(&sample_rate, &val, sizeof(float));
    val = data.at(6) | (data.at(7) << 16);
    memcpy(&exposure, &val, sizeof(float));

    if (isnan(dist)) dist = -1.;
    
    return;
}

}