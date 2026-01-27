#include "labkit/comms/usbtmccomm.hh"
#include <labkit/devices/rohde-schwarz/hmp4000.hh>
#include <labkit/utils.hh>
#include <labkit/exceptions.hh>
#include <labkit/debug.hh>

#include <unistd.h>
#include <sstream>

using namespace std;

namespace labkit
{

Hmp4000::Hmp4000(unique_ptr<TcpipComm> t_tcpip) : Hmp4000()
{
    this->connect(std::move(t_tcpip));
    return;
}

Hmp4000::Hmp4000(unique_ptr<SerialComm> t_serial) : Hmp4000()
{
    this->connect(std::move(t_serial));
    return;
}

Hmp4000::Hmp4000(unique_ptr<UsbTmcComm> t_usbtmc) : Hmp4000()
{
    this->connect(std::move(t_usbtmc));
    return;
}

void Hmp4000::connect(unique_ptr<TcpipComm> t_tcpip)
{
    this->setComm(std::move(t_tcpip));
    this->init();
    return;
}

void Hmp4000::connect(unique_ptr<SerialComm> t_serial)
{
    this->setComm(std::move(t_serial));
    this->init();
    return;
}

void Hmp4000::connect(unique_ptr<UsbTmcComm> t_usbtmc)
{
    t_usbtmc->configInterface(0);
    t_usbtmc->configEndpointIn(0x81);
    t_usbtmc->configEndpointOut(0x01);
    this->setComm(std::move(t_usbtmc));
    this->init();
    return;
}

void Hmp4000::enableChannel(unsigned t_channel, bool t_enable)
{
    this->selectChannel(t_channel);

    stringstream msg("");
    msg << "OUTP " << (t_enable ? "1" : "0") << "\n";
    this->getComm()->write(msg.str());
    return;
}

bool Hmp4000::channelEnabled(unsigned t_channel)
{
    this->selectChannel(t_channel);

    string resp = this->getComm()->query("OUTP?\n");
    return convertTo<bool>(resp);
}

void Hmp4000::setVoltage(unsigned t_channel, double t_volts)
{
    this->selectChannel(t_channel);

    stringstream msg("");
    msg << "VOLT " << t_volts << "\n";
    this->getComm()->write(msg.str());
    return;
}

double Hmp4000::getVoltage(unsigned t_channel)
{
    this->selectChannel(t_channel);

    string resp = this->getComm()->query("VOLT?\n");
    return convertTo<double>(resp);
}

double Hmp4000::measureVoltage(unsigned t_channel)
{
    this->selectChannel(t_channel);

    string resp = this->getComm()->query("MEAS:VOLT?\n");
    return convertTo<double>(resp);
}

void Hmp4000::setCurrent(unsigned t_channel, double t_amps)
{
    this->selectChannel(t_channel);

    stringstream msg("");
    msg << "CURR " << t_amps << "\n";
    this->getComm()->write(msg.str());
    return;
}

double Hmp4000::getCurrent(unsigned t_channel)
{
    this->selectChannel(t_channel);

    string resp = this->getComm()->query("CURR?\n");
    return convertTo<double>(resp);
}

double Hmp4000::measureCurrent(unsigned t_channel)
{
    this->selectChannel(t_channel);

    string resp = this->getComm()->query("MEAS:CURR?\n");
    return convertTo<double>(resp);
}

/*
 *      P R I V A T E   M E T H O D S
 */

void Hmp4000::init()
{
    m_scpi.setComm(this->getComm());
    m_scpi.cls();
    usleep(100e3);  // takes some time...

    // Get identifier and remove all control chars (i.e. /n, /0, etc.)
    m_name = removeCtrlChars(m_scpi.getIdn());
    return;
}

void Hmp4000::selectChannel(unsigned t_channel)
{
    if ( !this->channelValid(t_channel) )
            throw DeviceError("Invalid channel number " + to_string(t_channel));

    if (t_channel == m_cur_channel) // Nothing to do!
        return;

    stringstream msg("");
    msg << "INST OUTP" << (t_channel + 1) << "\n";
    DEBUG_PRINT("Switching to channel %u\n", t_channel);
    this->getComm()->write(msg.str());
    m_cur_channel = t_channel;
    return;
}

}