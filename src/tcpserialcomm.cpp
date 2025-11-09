#include <labkit/comms/tcpipserialcomm.hh>
#include <labkit/exceptions.hh>
#include <labkit/debug.hh>

#include <string>
#include <sstream>
#include <unistd.h>

using namespace std;

namespace labkit {

TcpipSerialComm::TcpipSerialComm(std::string t_ip_addr, unsigned t_port, 
    BaudRate t_baud, CharSize t_csize, Parity t_par, StopBits t_sbits) 
      : TcpipSerialComm()
{
    this->open(t_ip_addr, t_port, t_baud, t_csize, t_par, t_sbits);
    return;
}

TcpipSerialComm::~TcpipSerialComm()
{
    if (this->good())
        this->close();
    return;
}


void TcpipSerialComm::open()
{
    this->open(m_ip_addr, m_port, m_baud, m_csize, m_par, m_sbits);
    return;
}

void TcpipSerialComm::open(std::string ip_addr, unsigned port, BaudRate t_baud, 
    CharSize t_csize, Parity t_par, StopBits t_sbits)
{
    this->setIp(ip_addr);
    this->setPort(port);

    // Config via http
    m_tcpip_cfg.open(m_ip_addr, HTTP_PORT);

    // Serial communication via "raw" tcpip
    m_tcpip_ser.open(m_ip_addr, m_port);

    this->disableRtsCts();
    this->setBaud(t_baud);
    this->setCharSize(t_csize);
    this->setParity(t_par);
    this->setStopBits(t_sbits);
    this->applySettings();

    m_good = m_tcpip_ser.good() && m_tcpip_ser.good();
    return;
}

void TcpipSerialComm::close()
{
    m_tcpip_ser.close();
    m_good = m_tcpip_ser.good();
    return;
}

int TcpipSerialComm::writeRaw(const uint8_t* data, size_t len)
{
    if (m_update_settings) 
        this->applySettings();
    return m_tcpip_ser.writeRaw(data, len);
}

int TcpipSerialComm::readRaw(uint8_t* data, size_t max_len, unsigned timeout_ms)
{
    if (m_update_settings) 
        this->applySettings();
    return m_tcpip_ser.readRaw(data, max_len, timeout_ms);
}

std::string TcpipSerialComm::getInfo() const noexcept
{
    int nbits {0};
    switch (m_csize)
    {
        case CHAR_5: nbits = 5; break;
        case CHAR_6: nbits = 6; break;
        case CHAR_7: nbits = 7; break;
        case CHAR_8: nbits = 8; break;
    }

    char cpar {};
    switch (m_par) 
    {
        case PAR_NONE: cpar = 'N'; break;
        case PAR_EVEN: cpar = 'E'; break;
        case PAR_ODD: cpar = 'O'; break;
    }

    // Format example: serial;192.168.1.100:5555;9600;8N1
    stringstream ret {""};
    ret  << "serial;" << m_ip_addr << ":" << m_port << ";" << m_baud << ";";
    ret << nbits << cpar << m_sbits;
    return ret.str();
}

void TcpipSerialComm::setIp(std::string ip_addr)
{
    m_ip_addr = ip_addr;
    m_tcpip_cfg.setIp(m_ip_addr);
    m_tcpip_ser.setIp(m_ip_addr);
    DEBUG_PRINT("Set ip address to %s\n", m_ip_addr.c_str());
    return;
}

void TcpipSerialComm::setPort(unsigned port)
{
    m_port = port;
    m_tcpip_ser.setPort(m_port);
    DEBUG_PRINT("Set port to %u\n", m_port);
    return;
}

void TcpipSerialComm::setBaud(BaudRate t_baud)
{
    m_baud = t_baud;
    DEBUG_PRINT("Set baudrate to %i\n", m_baud);
    return;
}


void TcpipSerialComm::setCharSize(CharSize t_csize)
{
    DEBUG_PRINT("Set number of bits to %i\n", cSizeToInt(t_csize));
    m_csize = t_csize;
    m_update_settings = true;
    return;
}

void TcpipSerialComm::setStopBits(StopBits t_sbits)
{
    DEBUG_PRINT("Set number of stop bits to %i\n", t_sbits);
    m_sbits = t_sbits;
    m_update_settings = true;
    return;
}

void TcpipSerialComm::setParity(Parity t_par)
{
    DEBUG_PRINT("Set parity to %s\n", parToStr(t_par).c_str());
    m_par = t_par;
    m_update_settings = true;
    return;
}

void TcpipSerialComm::applySettings()
{
    DEBUG_PRINT("%s\n", "Applying settings to server");

    // http body
    string body("");
    body += "bdr=" + this->getBdr(m_baud) + "&";
    body += "dtb=" + to_string(this->getDtb(m_csize)) + "&";
    body += "prt=" + to_string(this->getPrt(m_par)) + "&";
    body += "stb=" + to_string(this->getStb(m_sbits)) + "&";
    body += "flc=" + to_string(m_flc) + "&";
    body += "rtp=&post=Submit";
    // http header
    string head = "POST /ok.html HTTP/1.1\r\n"
                  "Content-Length: " + to_string(body.size()) + "\r\n"
                  "\r\n";

    // TCP/IP server needs to restart
    m_tcpip_ser.close();

    m_tcpip_cfg.write(head + body);
    string ret = m_tcpip_cfg.readUntil("</SCRIPT>");   // End of message
    if ( ret.find("OK") == string::npos )
        throw BadProtocol("Did not receive 'HTTP/1.1 200 OK'");
    m_tcpip_cfg.close();

    // Reconnect to server
    usleep(100e3);
    m_tcpip_ser.open();
    m_tcpip_cfg.open();

    m_update_settings = false;
    return;
}

void TcpipSerialComm::enableRtsCts()
{
    DEBUG_PRINT("%s\n", "Enabled RTS/CTS flow control");
    m_flc = 1;
    return;
}

void TcpipSerialComm::disableRtsCts()
{
    DEBUG_PRINT("%s\n", "Disabled hardware flow control");
    m_flc = 0;
    return;
}

void TcpipSerialComm::enableDtrDsr()
{
    DEBUG_PRINT("%s\n", "Enabled DTR/DSR flow control");
    m_flc = 2;
    return;
}

void TcpipSerialComm::disableDtrDsr()
{
    DEBUG_PRINT("%s\n", "Disabled hardware flow control");
    m_flc = 0;
    return;
}

void TcpipSerialComm::enableXOnXOff(char xon, char xoff)
{
    throw Exception("Software flow control XON/XOFF is not supported by EthSerialComm");
    return;
}

void TcpipSerialComm::disableXOnXOff()
{
    DEBUG_PRINT("%s\n", "Disabled hardware flow control");
    m_flc = 0;
    return;
}

void TcpipSerialComm::setDtr()
{
    throw Exception("Setting DTR is currently not supported by EthSerialComm");
    return;
}

void TcpipSerialComm::clearDtr()
{
    throw Exception("Clearing DTR is currently not supported by EthSerialComm");
    return;
}

void TcpipSerialComm::setRts()
{
    throw Exception("Setting RTS is currently not supported by EthSerialComm");
    return;
}

void TcpipSerialComm::clearRts()
{
    throw Exception("Clearing RTS is currently not supported by EthSerialComm");
    return;
}

/*
 *      P R I V A T E   M E T H O D S
 */

string TcpipSerialComm::getBdr(BaudRate t_baud)
{
    string bdr = "0";
    switch(t_baud) {
        case BAUD_1200:   bdr = "0"; break;
        case BAUD_2400:   bdr = "1"; break;
        case BAUD_4800:   bdr = "2"; break;
        case BAUD_9600:   bdr = "4"; break;
        case BAUD_19200:  bdr = "6"; break;
        case BAUD_38400:  bdr = "8"; break;
        case BAUD_57600:  bdr = "9"; break;
        case BAUD_115200: bdr = "B"; break;

        default:
        throw DeviceError("Baudrate " + to_string(t_baud) + " is not supported\n");
    }
    return bdr;
}

unsigned TcpipSerialComm::getDtb(CharSize t_csize)
{
    unsigned dtb = 0;
    switch(t_csize) {
        case CHAR_8: dtb = 0; break;
        case CHAR_7: dtb = 1; break;
        case CHAR_6: dtb = 2; break;
        case CHAR_5: dtb = 3; break;
        default:
        throw DeviceError(to_string(cSizeToInt(t_csize)) + "-bit format is not supported\n");
    }
    return dtb;
}

unsigned TcpipSerialComm::getPrt(Parity t_par)
{
    unsigned prt = 0;
    switch (t_par)
    {
        case PAR_NONE: prt = 0; break;
        case PAR_ODD:  prt = 1; break;
        case PAR_EVEN: prt = 2; break; 
    }
    return prt;
}

unsigned TcpipSerialComm::getStb(StopBits t_sbits)
{
    unsigned stb = 0;
    switch (t_sbits) {
        case STOP_1: stb = 0; break;
        case STOP_2: stb = 1; break;
    }
    return stb;
}

}