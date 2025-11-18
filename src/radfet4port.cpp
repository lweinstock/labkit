#include <labkit/devices/jgu/radfet4port.hh>
#include <labkit/utils.hh>
#include <labkit/exceptions.hh>

#include <sstream>

namespace labkit
{

RadFET4Port::RadFET4Port(std::unique_ptr<TcpipComm> t_tcpip) : RadFET4Port()
{
    this->connect(std::move(t_tcpip));
    return;
}

void RadFET4Port::connect(std::unique_ptr<TcpipComm> t_tcpip)
{
    this->setComm(std::move(t_tcpip));
    this->init();
    return;
}

float RadFET4Port::getFETVoltage(unsigned t_channel, uint32_t &t_raw_adc)
{
    if (t_channel > 3)
        throw DeviceError("Invalid channel (0-3)");

    auto resp = this->getComm()->query("dose " + std::to_string(t_channel));

    // Extract voltage and raw ADC count from response
    double voltage {0};
    std::stringstream tmp(resp);
    tmp >> voltage >> t_raw_adc;
    return voltage;
}

float RadFET4Port::getFETVoltage(unsigned t_channel)
{
    uint32_t dummy {0x0000};
    return this->getFETVoltage(t_channel, dummy);
}

float RadFET4Port::getDiodeVoltage(unsigned t_channel, uint32_t &t_raw_adc)
{
    if (t_channel > 3)
        throw DeviceError("Invalid channel (0-3)");
    
    auto resp = this->getComm()->query("temp " + std::to_string(t_channel));
    
    // Extract voltage and raw ADC count from response
    double voltage {0};
    std::stringstream tmp(resp);
    tmp >> voltage >> t_raw_adc;
    return voltage;
}

float RadFET4Port::getDiodeVoltage(unsigned t_channel)
{
    uint32_t dummy {0x0000};
    return this->getDiodeVoltage(t_channel, dummy);
}

void RadFET4Port::reboot()
{
    this->getComm()->write("reboot");
    return;
}

/*
 *      P R I V A T E   M E T H O D S
 */

 void RadFET4Port::init()
 {
    auto id = this->getComm()->query("id\n");
    auto ver = this->getComm()->query("ver\n");
    auto build = this->getComm()->query("build\n");
    m_name = id + "," + ver + "," + build;
    m_name = removeCtrlChars(m_name);
    return;
 }

}