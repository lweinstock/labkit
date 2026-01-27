#include <labkit/devices/rigol/dg4000.hh>
#include <labkit/utils.hh>
#include <labkit/exceptions.hh>

#include <string>
#include <unistd.h>
#include <sstream>

using namespace std;

namespace labkit
{

Dg4000::Dg4000(std::unique_ptr<TcpipComm> t_tcpip) : Dg4000()
{
    this->connect(std::move(t_tcpip));
    return;
}

Dg4000::Dg4000(std::unique_ptr<UsbTmcComm> t_usbtmc) : Dg4000()
{
    this->connect(std::move(t_usbtmc));
    return;
}

void Dg4000::connect(std::unique_ptr<TcpipComm> t_tcpip)
{
    this->setComm(std::move(t_tcpip));
    this->init();
    return;
}

void Dg4000::connect(std::unique_ptr<UsbTmcComm> t_usbtmc)
{
    t_usbtmc->configInterface(0);
    t_usbtmc->configEndpointIn(0x86);
    t_usbtmc->configEndpointOut(0x02);
    this->setComm(std::move(t_usbtmc));
    this->init();
    return;
}

void Dg4000::enableChannel(unsigned t_channel, bool t_enable)
{
    if ( !this->channelValid(t_channel) )
        throw DeviceError("Invalid channel number " + to_string(t_channel));

    stringstream msg("");
    msg << ":OUTP" << (t_channel + 1) << ":STAT" << (t_enable? " ON" : " OFF") << "\n";
    this->getComm()->write(msg.str());
    return;
}

bool Dg4000::channelEnabled(unsigned t_channel)
{
    if ( !this->channelValid(t_channel) )
        throw DeviceError("Invalid channel number " + to_string(t_channel));
    
    stringstream msg("");
    msg << ":OUTP" << (t_channel + 1) << ":STAT?\n";
    string resp = this->getComm()->query(msg.str());
    if ( resp.find("ON") != string::npos )
        return true;
    return false;
}

void Dg4000::setWaveform(unsigned t_channel, Waveform t_wvfm)
{
    if ( !this->channelValid(t_channel) )
        throw DeviceError("Invalid channel number " + to_string(t_channel));
    
    stringstream msg("");
    msg << ":SOUR" << (t_channel + 1) << ":APPL:" << wvfmToString(t_wvfm) << "\n";
    this->getComm()->write(msg.str());
    return;
}

FunctionGenerator::Waveform Dg4000::getWaveform(unsigned t_channel)
{
    if ( !this->channelValid(t_channel) )
        throw DeviceError("Invalid channel number " + to_string(t_channel));
    
    // TODO!

    return SINE;
}

void Dg4000::setFrequency(unsigned t_channel, double t_freq_hz)
{
    if ( !this->channelValid(t_channel) )
        throw DeviceError("Invalid channel number " + to_string(t_channel));
    
    if (t_freq_hz < 0)
        throw DeviceError("Invalid frequency " + to_string(t_freq_hz) + "Hz");

    stringstream msg("");
    msg << ":SOUR" << (t_channel + 1) << ":FREQ " << t_freq_hz << "\n";
    this->writeAtLeast(msg.str(), 5);
    return;
}

double Dg4000::getFrequency(unsigned t_channel)
{
    if ( !this->channelValid(t_channel) )
        throw DeviceError("Invalid channel number " + to_string(t_channel));
    
    stringstream msg("");
    msg << ":SOUR" << (t_channel + 1) << ":FREQ?\n";
    string resp = this->getComm()->query(msg.str());
    return convertTo<double>(resp);
}

void Dg4000::setDutyCycle(unsigned t_channel, double t_dcl)
{
    if ( !this->channelValid(t_channel) )
        throw DeviceError("Invalid channel number " + to_string(t_channel));
    
    if ( (t_dcl < 0.) || (t_dcl > 1.) )
        throw DeviceError("Invalid duty cycle " + to_string(t_dcl));

    stringstream msg("");
    msg << ":SOUR" << (t_channel + 1) << ":PULS:DCYC " << 100*t_dcl << "\n";
    this->writeAtLeast(msg.str(), 5);
    return;
}

double Dg4000::getDutyCycle(unsigned t_channel)
{
    if ( !this->channelValid(t_channel) )
        throw DeviceError("Invalid channel number " + to_string(t_channel));
    
    stringstream msg("");
    msg << ":SOUR" << (t_channel + 1) << ":PULS:DCYC?\n";
    string resp = this->getComm()->query(msg.str());
    return convertTo<double>(resp)/100.;
}

void Dg4000::setPhase(unsigned t_channel, double t_phase_deg)
{
    if ( !this->channelValid(t_channel) )
        throw DeviceError("Invalid channel number " + to_string(t_channel));

    if ( (t_phase_deg < 0.) || (t_phase_deg > 360.) )
        throw DeviceError("Invalid phase angle " + to_string(t_phase_deg));

    stringstream msg("");
    msg << ":SOUR" << (t_channel + 1) << ":PHAS " << t_phase_deg << "\n";
    this->writeAtLeast(msg.str(), 5);    
    return;
}

double Dg4000::getPhase(unsigned t_channel)
{
    if ( !this->channelValid(t_channel) )
        throw DeviceError("Invalid channel number " + to_string(t_channel));

    stringstream msg("");
    msg << ":SOUR" << (t_channel + 1) << ":PHAS?\n";
    string resp = this->getComm()->query(msg.str());
    return convertTo<double>(resp);
}

void Dg4000::setAmplitude(unsigned t_channel, double t_ampl_v)
{
    if ( !this->channelValid(t_channel) )
        throw DeviceError("Invalid channel number " + to_string(t_channel));
    
    stringstream msg("");
    msg << ":SOUR" << (t_channel + 1) << ":VOLT " << t_ampl_v << "\n";
    this->writeAtLeast(msg.str(), 5);
    return;
}

double Dg4000::getAmplitude(unsigned t_channel)
{
    if ( !this->channelValid(t_channel) )
        throw DeviceError("Invalid channel number " + to_string(t_channel));
 
    stringstream msg("");
    msg << ":SOUR" << (t_channel + 1) << ":VOLT?\n";
    string resp = this->getComm()->query(msg.str());
    return convertTo<double>(resp);
}

void Dg4000::setOffset(unsigned t_channel, double t_offset_v)
{
    if ( !this->channelValid(t_channel) )
        throw DeviceError("Invalid channel number " + to_string(t_channel));
 
    stringstream msg("");
    msg << ":SOUR" << (t_channel + 1) << ":VOLT:OFFS " << t_offset_v << "\n";
    this->writeAtLeast(msg.str(), 5);
    return;
}

double Dg4000::getOffset(unsigned t_channel)
{
    if ( !this->channelValid(t_channel) )
        throw DeviceError("Invalid channel number " + to_string(t_channel));
    
    stringstream msg("");
    msg << ":SOUR" << (t_channel + 1) << ":VOLT:OFFS?\n";
    string resp = this->getComm()->query(msg.str());
    return convertTo<double>(resp);
}

void Dg4000::setRisingEdge(unsigned t_channel, double t_rise_s)
{
    if ( !this->channelValid(t_channel) )
        throw DeviceError("Invalid channel number " + to_string(t_channel));
 
    stringstream msg("");
    msg << ":SOUR" << (t_channel + 1) << ":PULS:TRAN:LEAD " << t_rise_s << "\n";
    this->getComm()->write(msg.str());
    return;
}

double Dg4000::getRisingEdge(unsigned t_channel)
{
    if ( !this->channelValid(t_channel) )
        throw DeviceError("Invalid channel number " + to_string(t_channel));
    
    stringstream msg("");
    msg << ":SOUR" << (t_channel + 1) << ":PULS:TRAN:LEAD?\n";
    string resp = this->getComm()->query(msg.str());
    return convertTo<double>(resp);
}

void Dg4000::setFallingEdge(unsigned t_channel, double t_fall_s)
{
    if ( !this->channelValid(t_channel) )
        throw DeviceError("Invalid channel number " + to_string(t_channel));

    stringstream msg("");
    msg << ":SOUR" << (t_channel + 1) << ":PULS:TRAN:TRA " << t_fall_s << "\n";
    this->getComm()->write(msg.str());
    return;
}

double Dg4000::getFallingEdge(unsigned t_channel)
{
    if ( !this->channelValid(t_channel) )
        throw DeviceError("Invalid channel number " + to_string(t_channel));
    
    stringstream msg("");
    msg << ":SOUR" << (t_channel + 1) << ":PULS:TRAN:TRA?\n";
    string resp = this->getComm()->query(msg.str());
    return convertTo<double>(resp);
}

void Dg4000::setPulseWidth(unsigned t_channel, double t_width_s)
{
    if ( !this->channelValid(t_channel) )
        throw DeviceError("Invalid channel number " + to_string(t_channel));
    
    stringstream msg("");
    msg << ":SOUR" << (t_channel + 1) << ":PULS:WIDT " << t_width_s << "\n";
    this->getComm()->write(msg.str());    
    return;
}

double Dg4000::getPulseWidth(unsigned t_channel)
{
    if ( !this->channelValid(t_channel) )
        throw DeviceError("Invalid channel number " + to_string(t_channel));

    stringstream msg("");
    msg << ":SOUR" << (t_channel + 1) << ":PULS:WIDT?\n";
    string resp = this->getComm()->query(msg.str());
    return stof(resp);    
    return 0.;
}

/*
 *      P R I V A T E   M E T H O D S
 */

void Dg4000::init() 
{
    m_scpi.setComm(this->getComm());
    m_scpi.cls();
    usleep(100e3);  // takes some time...

    // Get identifier and remove all control chars (i.e. /n, /0, etc.)
    m_name = removeCtrlChars(m_scpi.getIdn());
    return;
}

std::string Dg4000::wvfmToString(Waveform t_wfvm)
{
    switch (t_wfvm)
    {
        case SINE:      return "SIN";
        case SQUARE:    return "SQU";
        case RAMP:      return "RAMP";
        case PULSE:     return "PULS";
        case NOISE:     return "NOIS";
        case DC:        return "DC";
    }
    return "NONE";  // Never reached
}

void Dg4000::writeAtLeast(std::string t_msg, unsigned t_time_ms)
{
    /*
     * Known issue: while processing queries the DG4000 apparently ignores
     *  all following queries instead of storing them in an internal command
     *  queue. This makes it neccesary for some write commands to take at
     *  least a few milliseconds before returning.
     */

    struct timeval sta, sto;
    gettimeofday(&sta, NULL);
    this->getComm()->write(t_msg);
    gettimeofday(&sto, NULL);

    unsigned diff_ms = (sto.tv_sec-sta.tv_sec)*1000 + (sto.tv_usec-sta.tv_usec)/1000;
    if (t_time_ms > diff_ms)
        usleep( (t_time_ms - diff_ms)*1000 );
    return;
}

}