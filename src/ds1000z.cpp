#include <labkit/devices/rigol/ds1000z.hh>
#include <labkit/exceptions.hh>
#include <labkit/utils.hh>
#include <labkit/debug.hh>

#include <memory>
#include <sstream>

using namespace std;

namespace labkit
{

Ds1000Z::Ds1000Z(unique_ptr<TcpipComm> t_tcpip) : Ds1000Z()
{
    this->connect(std::move(t_tcpip));
    return;
}

Ds1000Z::Ds1000Z(unique_ptr<UsbTmcComm> t_usbtmc) : Ds1000Z()
{
    this->connect(std::move(t_usbtmc));
    return;
}

void Ds1000Z::connect(unique_ptr<TcpipComm> t_tcpip)
{
    this->setComm(std::move(t_tcpip));
    this->init();
    return;
}

void Ds1000Z::connect(unique_ptr<UsbTmcComm> t_usbtmc)
{
    t_usbtmc->configInterface(0);
    t_usbtmc->configEndpointIn(0x82);
    t_usbtmc->configEndpointOut(0x03);
    this->setComm(std::move(t_usbtmc));
    this->init();
    return;
}

void Ds1000Z::enableChannel(unsigned t_channel, bool t_enable)
{
    if ( !this->channelValid(t_channel) )
        throw DeviceError("Invalid channel number " + to_string(t_channel));

    stringstream msg("");
    msg << ":CHAN" << (t_channel + 1) << ":DISP ";
    if (t_enable) msg << "1\n";
    else msg << "0\n";
    this->getComm()->write(msg.str());
    return;
}

bool Ds1000Z::channelEnabled(unsigned t_channel)
{
    if ( !this->channelValid(t_channel) )
        throw DeviceError("Invalid channel number " + to_string(t_channel));

    stringstream msg("");
    msg << ":CHAN" << (t_channel + 1) << ":DISP?\n";
    string resp = this->getComm()->query(msg.str());
    return convertTo<bool>(resp);
}

void Ds1000Z::setAtten(unsigned t_channel, double t_att)
{
    if ( !this->channelValid(t_channel) )
        throw DeviceError("Invalid channel number " + to_string(t_channel));

    stringstream msg("");
    msg << ":CHAN" << (t_channel + 1) << ":PROB " << t_att << "\n";
    this->getComm()->write(msg.str());
    return;
}

double Ds1000Z::getAtten(unsigned t_channel)
{
    if ( !this->channelValid(t_channel) )
        throw DeviceError("Invalid channel number " + to_string(t_channel));

    stringstream msg("");
    msg << ":CHAN" << (t_channel + 1) << ":PROB?\n";
    string resp = this->getComm()->query(msg.str());
    return convertTo<double>(resp);
}

void Ds1000Z::setVertBase(unsigned t_channel, double t_volts_per_div)
{
    if ( !this->channelValid(t_channel) )
        throw DeviceError("Invalid channel number " + to_string(t_channel));

    stringstream msg("");
    msg << ":CHAN" << (t_channel + 1) << ":SCAL " << t_volts_per_div << "\n";
    this->getComm()->write(msg.str());
    return;
}

double Ds1000Z::getVertBase(unsigned t_channel)
{
    if ( !this->channelValid(t_channel) )
        throw DeviceError("Invalid channel number " + to_string(t_channel));

    stringstream msg("");
    msg << ":CHAN" << (t_channel + 1) << ":SCAL?\n";
    string resp = this->getComm()->query(msg.str());
    return convertTo<double>(resp);
}

void Ds1000Z::setVertOffset(unsigned t_channel, double t_offset_v)
{
    if ( !this->channelValid(t_channel) )
        throw DeviceError("Invalid channel number " + to_string(t_channel));

    stringstream msg("");
    msg << ":CHAN" << (t_channel + 1) << ":OFFS " << t_offset_v << "\n";
    this->getComm()->write(msg.str());
    return;
}

double Ds1000Z::getVertOffset(unsigned t_channel)
{
    if ( !this->channelValid(t_channel) )
        throw DeviceError("Invalid channel number " + to_string(t_channel));

    stringstream msg("");
    msg << ":CHAN" << (t_channel + 1) << ":OFFS?\n";
    string resp = this->getComm()->query(msg.str());
    return convertTo<double>(resp);
}

void Ds1000Z::setHorzBase(double t_sec_per_div)
{
    stringstream msg("");
    msg << ":TIM:SCAL " << t_sec_per_div << "\n";
    this->getComm()->write(msg.str());
    return;
}
double Ds1000Z::getHorzBase()
{
    string resp = this->getComm()->query(":TIM:SCAL?\n");
    return convertTo<double>(resp);
}

void Ds1000Z::setHorzOffset(double t_offset_s)
{
    stringstream msg("");
    msg << ":TIM:OFFS " << t_offset_s << "\n";
    this->getComm()->write(msg.str());
    return;
}

double Ds1000Z::getHorzOffset()
{
    string resp = this->getComm()->query(":TIM:OFFS?\n");
    return convertTo<double>(resp);
}

void Ds1000Z::setMeasurement(unsigned t_channel, MeasurementItem t_meas)
{
    if ( !this->channelValid(t_channel) )
        throw DeviceError("Invalid channel number " + to_string(t_channel));

    // Valid single source measurement?
    switch (t_meas) {
        case VMAX: 
        case VMIN:
        case VPP:
        case VTOP:
        case VBASE:
        case VAMP:
        case VAVG:
        case VRMS:
        case OVERSHOOT:
        case PRESHOOT:
        case FREQ:
        case RISETIME:
        case FALLTIME:
        case POS_WIDTH:
        case NEG_WIDTH:
        case POS_DUTY:
        case NEG_DUTY:
        break;

        default:
        throw DeviceError("Invalid single source measurement " 
            + measToString(t_meas));
    }

    stringstream msg("");
    msg << ":MEAS:ITEM " << measToString(t_meas) << ",CHAN" << (t_channel + 1) << "\n";
    this->getComm()->write(msg.str());
    return;
}

double Ds1000Z::getMeasurement(unsigned t_channel, MeasurementItem t_meas)
{
    if ( !this->channelValid(t_channel) )
        throw DeviceError("Invalid channel number " + to_string(t_channel));

    // Valid single source measurement?
    switch (t_meas) {
        case VMAX: 
        case VMIN:
        case VPP:
        case VTOP:
        case VBASE:
        case VAMP:
        case VAVG:
        case VRMS:
        case OVERSHOOT:
        case PRESHOOT:
        case FREQ:
        case RISETIME:
        case FALLTIME:
        case POS_WIDTH:
        case NEG_WIDTH:
        case POS_DUTY:
        case NEG_DUTY:
        break;

        default:
        throw DeviceError("Invalid single source measurement " 
            + measToString(t_meas));
    }

    stringstream msg("");
    msg << ":MEAS:ITEM? " << measToString(t_meas) << ",CHAN" << (t_channel + 1) << "\n";
    string resp = this->getComm()->query(msg.str());
    return convertTo<double>(resp);
}

void Ds1000Z::setMeasurement(unsigned t_channel1, unsigned t_channel2, 
    MeasurementItem t_meas)
{
    if ( !this->channelValid(t_channel1) )
        throw DeviceError("Invalid channel number " + to_string(t_channel1));

    if ( !this->channelValid(t_channel2) )
        throw DeviceError("Invalid channel number " + to_string(t_channel2));

    // Valid dual source measurement?
    switch (t_meas) {
        case POS_DELAY:
        case NEG_DELAY:
        case POS_PHASE:
        case NEG_PHASE:
        break;

        default:
        throw DeviceError("Invalid dual source measurement " 
            + measToString(t_meas));
    }
    
    stringstream msg("");
    msg << ":MEAS:ITEM " << measToString(t_meas) << ",CHAN" << t_channel1;
    msg << ",CHAN" << t_channel2 << "\n";
    this->getComm()->write(msg.str());
    return;
}

double Ds1000Z::getMeasurement(unsigned t_channel1, unsigned t_channel2, 
    MeasurementItem t_meas)
{
    if ( !this->channelValid(t_channel1) )
        throw DeviceError("Invalid channel number " + to_string(t_channel1));
    
    if ( !this->channelValid(t_channel2) )
        throw DeviceError("Invalid channel number " + to_string(t_channel2));
    
    // Valid dual source measurement?
    switch (t_meas) {
        case POS_DELAY:
        case NEG_DELAY:
        case POS_PHASE:
        case NEG_PHASE:
        break;

        default:
        throw DeviceError("Invalid dual source measurement " 
            + measToString(t_meas));
    }

    stringstream msg("");
    msg << ":MEAS:ITEM? " << measToString(t_meas) << ",CHAN" << t_channel1;
    msg << ",CHAN" << t_channel2 << "\n";
    string resp = this->getComm()->query(msg.str());
    return convertTo<double>(resp);
}

void Ds1000Z::clearMeasurements()
{
    this->getComm()->write(":MEAS:CLE\n");
    return;
}

void Ds1000Z::run()
{
    this->getComm()->write(":RUN\n");
    return;
}

void Ds1000Z::stop()
{
    this->getComm()->write(":STOP\n");
    return;
}

void Ds1000Z::singleShot()
{
    this->getComm()->write(":SING\n");
    return;
}

void Ds1000Z::setTriggerType(TriggerType t_trig)
{
    stringstream msg("");
    msg << ":TRIG:MODE EDGE\n";
    this->getComm()->write(msg.str().c_str());

    msg.str("");
    msg << ":TRIG:EDG:SLOP " << this->trigToString(t_trig) << "\n";
    this->getComm()->write(msg.str().c_str());
    return;
}

void Ds1000Z::setTriggerLevel(double t_level)
{
    stringstream msg("");
    msg << ":TRIG:EDG:LEV " << t_level << "\n";
    this->getComm()->write(msg.str().c_str());
    return;
}

void Ds1000Z::setTriggerSource(unsigned t_channel)
{
    stringstream msg("");
    msg << ":TRIG:EDG:SOUR CHAN" << (t_channel + 1) << "\n";
    this->getComm()->write(msg.str().c_str());
    return;
}

bool Ds1000Z::triggered()
{
    string resp = this->getComm()->query(":TRIG:STAT?\n");
    if ( resp.find("TD") != string::npos )
        return true;
    return false;
}

bool Ds1000Z::stopped()
{
    string status = this->getComm()->query(":TRIG:STAT?\n");
    if ( status.find("STOP") != string::npos )
        return true;
    return false;
}

void Ds1000Z::readSampleData(unsigned t_channel, vector<double> &t_horz_data, 
    vector<double> &t_vert_data)
{
    if ( !this->channelValid(t_channel) )
        throw DeviceError("Invalid channel number " + to_string(t_channel));

    this->getComm()->write(":WAV:SOUR CHAN" + to_string(t_channel) + "\n");

    // Get waveform preamble
    string data = this->getComm()->query(":WAV:PRE?\n");
    vector<string> preamble = split(data, ",");
    if (preamble.size() != 10) {
        DEBUG_PRINT("Received wrong preamble size (%lu): '%s'\n",
            preamble.size(), data.c_str());
        throw DeviceError("Received incomplete preamble.");
    }

    // Extract data from preamble
    unsigned npts  = stoi( preamble.at(2) );
    double xincr = stof( preamble.at(4) );
    double xorg  = stof( preamble.at(5) );
    double xref  = stof( preamble.at(6) );
    double yinc  = stof( preamble.at(7) );
    int yorg  = stoi( preamble.at(8) );
    int yref  = stoi( preamble.at(9) );

    DEBUG_PRINT("pts = %i\n", npts);
    DEBUG_PRINT("xincr = %e\n", xincr);
    DEBUG_PRINT("xorg = %e\n", xorg);
    DEBUG_PRINT("xref = %e\n", xref);
    DEBUG_PRINT("yinc = %e\n", yinc);
    DEBUG_PRINT("yorg = %i\n", yorg);
    DEBUG_PRINT("yref = %i\n", yref);

    // Read waveform in chunks of 250kSa
    vector<uint8_t> mem_data, temp;
    unsigned start = 1, stop = 250000;
    while (mem_data.size() < npts) {
        this->setMemoryDataRange(start, stop);
        temp = this->readMemoryData();
        start += temp.size();
        stop += temp.size();
        if (stop > npts) stop = npts;
        mem_data.insert(mem_data.end(), temp.begin(), temp.end());
        usleep(10e3);  // 10ms wait to avoid accessive polling
    }
    DEBUG_PRINT("Total points read from memory: %lu\n", mem_data.size());

    // Convert byte data using preamble
    t_horz_data.resize(npts);
    t_vert_data.resize(npts);
    double xval, yval;
    for (size_t i = 0; i < npts; i++) {
        xval = i*xincr + xorg; 
        yval = (mem_data.at(i) - yref - yorg) *  yinc;
        t_vert_data[i] = xval;
        t_horz_data[i] = yval;
    }
    return;
}

/*
 *      P R I V A T E   M E T H O D S
 */

void Ds1000Z::init()
{
    m_scpi.setComm(this->getComm());
    m_scpi.cls();
    usleep(100e3);
    // Get identifier and remove all control chars (i.e. /n, /0, etc.)
    m_name = removeCtrlChars(m_scpi.getIdn());

    // Set waveform format
    this->getComm()->write(":WAV:FORM BYTE\n");
    this->getComm()->write(":WAV:MODE MAX\n");
    return;
}

void Ds1000Z::setMemoryDataRange(unsigned t_sta, unsigned t_sto)
{    
    // Set start and stop address
    this->getComm()->write(":WAV:STAR " + to_string(t_sta) + "\n");
    this->getComm()->write(":WAV:STOP " + to_string(t_sto) + "\n");
    return;
}

vector<uint8_t> Ds1000Z::readMemoryData() 
{
    // Read data block defined by set_mem_range
    string data = this->getComm()->query(":WAV:DATA?\n");
    // Extract header
    size_t len = 0;
    string header = data.substr(0, 11);
    sscanf(header.c_str(), "#9%9zd", &len);
    DEBUG_PRINT("len = %zu\n", len);

    // Read the waveform
    while (data.size() < len)
        data.append( this->getComm()->read() );

    vector<uint8_t> ret(len);
    for (size_t i = 0; i < len; i++)
        ret[i] = (uint8_t)data.at(i + header.size());

    return ret;   
}

string Ds1000Z::measToString(MeasurementItem t_meas)
{
    switch (t_meas)
    {
        case VMAX:      return "VMAX";
        case VMIN:      return "VMIN";
        case VPP:       return "VPP";
        case VTOP:      return "VTOP";
        case VBASE:     return "VBAS";
        case VAMP:      return "VAMP";
        case VAVG:      return "VAVG";
        case VRMS:      return "VRMS";        
        case OVERSHOOT: return "OVER";
        case PRESHOOT:  return "PRES";
        case FREQ:      return "FREQ";        
        case RISETIME:  return "RTIM";
        case FALLTIME:  return "FTIM";
        case POS_WIDTH: return "PWID";        
        case NEG_WIDTH: return "NWID";
        case POS_DUTY:  return "PDUT";
        case NEG_DUTY:  return "NDUT";        
        case POS_DELAY: return "RDEL";
        case NEG_DELAY: return "FDEL";
        case POS_PHASE: return "RPH";        
        case NEG_PHASE: return "FPH";
    }
    return "NONE";    // Never reached
}

std::string Ds1000Z::trigToString(TriggerType t_trig)
{
    switch (t_trig) {
        case RISE:  return "POS";
        case FALL:  return "NEG";
        case BOTH:  return "RFAL";
    }
    return "NONE";  // Never reached
}


}