#include <ios>
#include <labkit/devices/siglent/sdg1000x.hh>
#include <labkit/utils.hh>
#include <labkit/exceptions.hh>

#include <string>
#include <unistd.h>
#include <sstream>
#include <iomanip>

using namespace std;

/* T O D O */

namespace labkit
{

Sdg1000X::Sdg1000X(std::unique_ptr<TcpipComm> t_tcpip) : Sdg1000X()
{
    this->connect(std::move(t_tcpip));
    return;
}

Sdg1000X::Sdg1000X(std::unique_ptr<UsbTmcComm> t_usbtmc) : Sdg1000X()
{
    this->connect(std::move(t_usbtmc));
    return;
}

void Sdg1000X::connect(std::unique_ptr<TcpipComm> t_tcpip)
{
    this->setComm(std::move(t_tcpip));
    this->init();
    return;
}

void Sdg1000X::connect(std::unique_ptr<UsbTmcComm> t_usbtmc)
{
    t_usbtmc->configInterface(0);
    t_usbtmc->configEndpointIn(0x81);
    t_usbtmc->configEndpointOut(0x01);
    this->setComm(std::move(t_usbtmc));
    this->init();
    return;
}

void Sdg1000X::enableChannel(unsigned t_channel, bool t_enable)
{
    if ( !this->channelValid(t_channel) )
        throw DeviceError("Invalid channel number " + to_string(t_channel));

    stringstream msg("");
    msg << "C" << t_channel << ":OUTP " << (t_enable ? "ON" : "OFF") << "\n";
    this->getComm()->write(msg.str());
    return;
}

bool Sdg1000X::channelEnabled(unsigned t_channel)
{
    if ( !this->channelValid(t_channel) )
        throw DeviceError("Invalid channel number " + to_string(t_channel));
    
    stringstream msg("");
    msg << "C" << t_channel << ":OUTP?\n";
    string resp = this->getComm()->query(msg.str());
    if ( resp.find("ON") != string::npos )
        return true;
    return false;
}

void Sdg1000X::setWaveform(unsigned t_channel, Waveform t_wvfm)
{
    if ( !this->channelValid(t_channel) )
        throw DeviceError("Invalid channel number " + to_string(t_channel));
    
    stringstream msg("");
    msg << "C" << t_channel << ":BSWV WVTP," << wvfmToString(t_wvfm) << "\n";
    this->getComm()->write(msg.str());
    return;
}

FunctionGenerator::Waveform Sdg1000X::getWaveform(unsigned t_channel)
{
    if ( !this->channelValid(t_channel) )
        throw DeviceError("Invalid channel number " + to_string(t_channel));
    
    // TODO!

    return SINE;
}

void Sdg1000X::setFrequency(unsigned t_channel, double t_freq_hz)
{
    if ( !this->channelValid(t_channel) )
        throw DeviceError("Invalid channel number " + to_string(t_channel));
    
    if (t_freq_hz < 0)
        throw DeviceError("Invalid frequency " + to_string(t_freq_hz) + "Hz");

    stringstream msg("");
    msg << "C" << t_channel << ":BSWV FRQ,";
    msg << setprecision(3) << fixed << t_freq_hz << "\n";
    this->getComm()->write(msg.str());
    return;
}

double Sdg1000X::getFrequency(unsigned t_channel)
{
    if ( !this->channelValid(t_channel) )
        throw DeviceError("Invalid channel number " + to_string(t_channel));
    
    stringstream msg("");
    msg << "C" << t_channel << ":BSWV?\n";
    string resp = this->getComm()->query(msg.str());
    return convertTo<double>(this->getBswvVal(resp, "FRQ"));
}

void Sdg1000X::setDutyCycle(unsigned t_channel, double t_dcl)
{
    if ( !this->channelValid(t_channel) )
        throw DeviceError("Invalid channel number " + to_string(t_channel));
    
    if ( (t_dcl < 0.) || (t_dcl > 1.) )
        throw DeviceError("Invalid duty cycle " + to_string(t_dcl));

    stringstream msg("");
    msg << "C" << t_channel << ":BSWV DUTY,";
    msg << setprecision(3) << fixed << 100.*t_dcl << "\n";
    this->getComm()->write(msg.str());
    return;
}

double Sdg1000X::getDutyCycle(unsigned t_channel)
{
    if ( !this->channelValid(t_channel) )
        throw DeviceError("Invalid channel number " + to_string(t_channel));
    
    stringstream msg("");
    msg << "C" << t_channel << ":BSWV?\n";
    string resp = this->getComm()->query(msg.str());
    return 0.01 * convertTo<double>(this->getBswvVal(resp, "FRQ"));
}

void Sdg1000X::setPhase(unsigned t_channel, double t_phase_deg)
{
    if ( !this->channelValid(t_channel) )
        throw DeviceError("Invalid channel number " + to_string(t_channel));

    if ( (t_phase_deg < 0.) || (t_phase_deg > 360.) )
        throw DeviceError("Invalid phase angle " + to_string(t_phase_deg));

    stringstream msg("");
    msg << "C" << t_channel << ":BSWV PHSE,";
    msg << setprecision(3) << fixed << t_phase_deg << "\n";
    this->getComm()->write(msg.str());
    return;
}

double Sdg1000X::getPhase(unsigned t_channel)
{
    if ( !this->channelValid(t_channel) )
        throw DeviceError("Invalid channel number " + to_string(t_channel));

    stringstream msg("");
    msg << "C" << t_channel << ":BSWV?\n";
    string resp = this->getComm()->query(msg.str());
    return convertTo<double>(this->getBswvVal(resp, "PHSE"));
}

void Sdg1000X::setAmplitude(unsigned t_channel, double t_ampl_v)
{
    if ( !this->channelValid(t_channel) )
        throw DeviceError("Invalid channel number " + to_string(t_channel));
    
    stringstream msg("");
    msg << "C" << t_channel << ":BSWV AMP,";
    msg << setprecision(3) << fixed << t_ampl_v << "\n";
    this->getComm()->write(msg.str());
    return;
}

double Sdg1000X::getAmplitude(unsigned t_channel)
{
    if ( !this->channelValid(t_channel) )
        throw DeviceError("Invalid channel number " + to_string(t_channel));
 
    stringstream msg("");
    msg << "C" << t_channel << ":BSWV?\n";
    string resp = this->getComm()->query(msg.str());
    return convertTo<double>(this->getBswvVal(resp, "AMP"));
}

void Sdg1000X::setOffset(unsigned t_channel, double t_offset_v)
{
    if ( !this->channelValid(t_channel) )
        throw DeviceError("Invalid channel number " + to_string(t_channel));
 
    stringstream msg("");
    msg << "C" << t_channel << ":BSWV OFST,";
    msg << setprecision(3) << fixed << t_offset_v << "\n";
    this->getComm()->write(msg.str());
    return;
}

double Sdg1000X::getOffset(unsigned t_channel)
{
    if ( !this->channelValid(t_channel) )
        throw DeviceError("Invalid channel number " + to_string(t_channel));
    
    stringstream msg("");
    msg << "C" << t_channel << ":BSWV?\n";
    string resp = this->getComm()->query(msg.str());
    return convertTo<double>(this->getBswvVal(resp, "OFST"));
}

void Sdg1000X::setRisingEdge(unsigned t_channel, double t_rise_s)
{
    if ( !this->channelValid(t_channel) )
        throw DeviceError("Invalid channel number " + to_string(t_channel));
 
    stringstream msg("");
    msg << "C" << t_channel << ":BSWV RISE,";
    msg << setprecision(3) << fixed << scientific << t_rise_s << "\n";
    this->getComm()->write(msg.str());
    return;
}

double Sdg1000X::getRisingEdge(unsigned t_channel)
{
    if ( !this->channelValid(t_channel) )
        throw DeviceError("Invalid channel number " + to_string(t_channel));
    
    stringstream msg("");
    msg << "C" << t_channel << ":BSWV?\n";
    string resp = this->getComm()->query(msg.str());
    return convertTo<double>(this->getBswvVal(resp, "RISE"));
}

void Sdg1000X::setFallingEdge(unsigned t_channel, double t_fall_s)
{
    if ( !this->channelValid(t_channel) )
        throw DeviceError("Invalid channel number " + to_string(t_channel));

    stringstream msg("");
    msg << "C" << t_channel << ":BSWV FALL,";
    msg << setprecision(3) << fixed << scientific << t_fall_s << "\n";
    this->getComm()->write(msg.str());
    return;
}

double Sdg1000X::getFallingEdge(unsigned t_channel)
{
    if ( !this->channelValid(t_channel) )
        throw DeviceError("Invalid channel number " + to_string(t_channel));
    
    stringstream msg("");
    msg << "C" << t_channel << ":BSWV?\n";
    string resp = this->getComm()->query(msg.str());
    return convertTo<double>(this->getBswvVal(resp, "FALL"));
}

void Sdg1000X::setPulseWidth(unsigned t_channel, double t_width_s)
{
    if ( !this->channelValid(t_channel) )
        throw DeviceError("Invalid channel number " + to_string(t_channel));
    
    stringstream msg("");
    msg << "C" << t_channel << ":BSWV WIDTH,";
    msg << setprecision(3) << fixed << scientific << t_width_s << "\n";
    this->getComm()->write(msg.str());
    return;
}

double Sdg1000X::getPulseWidth(unsigned t_channel)
{
    if ( !this->channelValid(t_channel) )
        throw DeviceError("Invalid channel number " + to_string(t_channel));

    stringstream msg("");
    msg << "C" << t_channel << ":BSWV?\n";
    string resp = this->getComm()->query(msg.str());
    return convertTo<double>(this->getBswvVal(resp, "WIDTH"));
}

/*
 *      P R I V A T E   M E T H O D S
 */

void Sdg1000X::init() 
{
    m_scpi.setComm(this->getComm());
    m_scpi.cls();
    usleep(100e3);  // takes some time...

    // Get identifier and remove all control chars (i.e. /n, /0, etc.)
    m_name = removeCtrlChars(m_scpi.getIdn());
    return;
}

std::string Sdg1000X::wvfmToString(Waveform t_wfvm)
{
    switch (t_wfvm)
    {
        case SINE:      return "SINE";
        case SQUARE:    return "SQUARE";
        case RAMP:      return "RAMP";
        case PULSE:     return "PULSE";
        case NOISE:     return "NOISE";
        case DC:        return "DC";
    }
    return "NONE";  // Never reached
}

std::string Sdg1000X::getBswvVal(const string &t_bswv, const string &t_par)
{
    size_t pos1 = t_bswv.find(t_par);
    if (pos1 == string::npos)   // Parameter not found
        throw BadProtocol("Could not find parameter '" + t_par 
            + "' in BSWV string '" + t_bswv + "'");
    pos1 += t_par.size() + 1;
    // Values and parameters are separated by commas
    size_t pos2 = t_bswv.find(',', pos1 + 1);
    return t_bswv.substr(pos1, pos2 - pos1);
} 

}