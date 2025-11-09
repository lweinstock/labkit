#include <labkit/comms/serialcomm.hh>
#include <labkit/exceptions.hh>
#include <labkit/debug.hh>

#include <fcntl.h>          // file control definitions
#include <string>
#include <unistd.h>         // open(), close(), read(), write(), ...
#include <errno.h>          // errno, strerr(), ...
#include <sys/ioctl.h>      // ioctl()
#include <sys/select.h>     // select()
#include <sstream>
#include <string.h>

using namespace std;

namespace labkit 
{

SerialComm::SerialComm(std::string t_path, BaudRate t_baud, CharSize t_csize,
    Parity t_par, StopBits t_sbits) : SerialComm()
{
    this->open(t_path, t_baud, t_csize, t_par, t_sbits);
    return;
}

SerialComm::~SerialComm() 
{
    if (this->good())
        this->close();
    return;
}

void SerialComm::open()
{
    this->open(m_path, m_baud, m_csize, m_par, m_sbits);
    return;
}

void SerialComm::open(std::string t_path, BaudRate t_baud, CharSize t_csize,
    Parity t_par, StopBits t_sbits)
{
    DEBUG_PRINT("Opening device '%s'\n", t_path.c_str());
    m_fd = ::open(t_path.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    checkAndThrow(m_fd, "Failed to open device " + t_path);
    m_path = t_path;

    int stat = tcgetattr(m_fd, &m_term_settings);
    checkAndThrow(stat, "Failed to get termios attributes from " + t_path);

    // Ignore modem control lines, enable receiver
    m_term_settings.c_cflag |= (CLOCAL | CREAD);

    // Input modes: no sw flow control, ignore break conditions
    m_term_settings.c_iflag &= ~(IXON | IXOFF | IXANY | IGNBRK);
    // Don't map NL (\n) to CR (\r) and vice versa, don't ignore CR
    m_term_settings.c_iflag &= ~(INLCR | ICRNL | IGNCR);

    // Output modes: no processing
    m_term_settings.c_oflag &= ~OPOST;

    // Local modes: disable canonical mode, no echo, erasure
    m_term_settings.c_lflag &= ~(ICANON | ECHO | ECHOE | ECHONL | ISIG);

    // Special characters: non-blocking I/O, no timeout
    m_term_settings.c_cc[VMIN] = 0;
    m_term_settings.c_cc[VTIME] = 0;

    this->disableRtsCts();
    this->setBaud(t_baud);
    this->setCharSize(t_csize);
    this->setParity(t_par);
    this->setStopBits(t_sbits);
    this->applySettings();

    m_good = true;
    return;
}

void SerialComm::close()
{
    if (!m_good) 
        return;

    DEBUG_PRINT("Closing device '%s'\n", m_path.c_str());
    int stat = ::close(m_fd);
    checkAndThrow(stat, "Failed to close '" + m_path + "'");
    m_good = false;
    return;
}

int SerialComm::writeRaw(const uint8_t* t_data, size_t t_len) 
{
    if (m_update_settings) this->applySettings();

    size_t bytes_left = t_len;
    size_t bytes_written = 0;
    ssize_t nbytes = 0;

    while ( bytes_left > 0 ) {
        nbytes = ::write(m_fd, &t_data[bytes_written], bytes_left);
        checkAndThrow(nbytes, "Failed to write to device");
        bytes_left -= nbytes;

        DEBUG_PRINT_BYTE_DATA(t_data, nbytes, "Written %zu bytes: ", nbytes);

        bytes_written += nbytes;
    }

    return bytes_written;
}

int SerialComm::readRaw(uint8_t* t_data, size_t t_max_len, unsigned t_timeout_ms) 
{
    if (m_update_settings) this->applySettings();

    // Wait for I/O
    fd_set rfd_set;
    FD_ZERO(&rfd_set);
    FD_SET(m_fd, &rfd_set);
    m_timeout.tv_sec = t_timeout_ms / 1000.;
    m_timeout.tv_usec = t_timeout_ms % 1000;

    // Block until data is available or timeout exceeded
    int stat = select(m_fd + 1, &rfd_set, NULL, NULL, &m_timeout);
    checkAndThrow(stat, "No data available");
    if (stat ==  0)
        throw Timeout("Read timeout occurred", errno);

    // Data is available!
    ssize_t nbytes;
    nbytes = ::read(m_fd, t_data, t_max_len);
    checkAndThrow(nbytes, "Failed to read from device");
    
    DEBUG_PRINT_BYTE_DATA(t_data, nbytes, "Read %zu bytes: ", nbytes);

    return nbytes;
}

string SerialComm::getInfo() const noexcept
{
    // Format example: serial;/dev/tty0;9600;8N1
    stringstream ret {""};
    ret  << "serial;" << m_path << ";" << m_baud << ";";
    ret << cSizeToInt(m_csize) << parToChar(m_par) << m_sbits;
    return ret.str();
}

void SerialComm::setBaud(BaudRate t_baud) 
{
    DEBUG_PRINT("Setting baudrate to %i\n", t_baud);

    int stat = cfsetispeed(&m_term_settings, t_baud);
    checkAndThrow(stat, "Failed to set in baudrate " + to_string(t_baud));
    stat = cfsetospeed(&m_term_settings, t_baud);
    checkAndThrow(stat, "Failed to set out baudrate " + to_string(t_baud));
    m_baud = t_baud;
    m_update_settings = true;
    return;
}

void SerialComm::setCharSize(CharSize t_csize) 
{
    m_term_settings.c_cflag &= ~CSIZE;
    m_term_settings.c_cflag |= t_csize;

    DEBUG_PRINT("Set number of bits to %i\n", cSizeToInt(t_csize));

    m_csize = t_csize;
    m_update_settings = true;
    return;
}

void SerialComm::setParity(Parity t_par) 
{
    switch (t_par)
    {
        case PAR_NONE:
        m_term_settings.c_cflag &= ~PARENB;
        break;

        case PAR_EVEN:
        m_term_settings.c_cflag |= PARENB;
        m_term_settings.c_cflag &= ~PARODD;
        break;

        case PAR_ODD:
        m_term_settings.c_cflag |= PARENB;
        m_term_settings.c_cflag |= PARODD;
        break;
    }

    DEBUG_PRINT("Set parity to %s\n", parToStr(t_par).c_str());

    m_par = t_par;
    m_update_settings = true;
    return;
}

void SerialComm::setStopBits(StopBits t_sbits) 
{
    switch (t_sbits) 
    {
        case STOP_1: m_term_settings.c_cflag &= ~CSTOPB; break;
        case STOP_2: m_term_settings.c_cflag |= CSTOPB;  break;
    }
    m_sbits = t_sbits;

    DEBUG_PRINT("set number of stop bits to %i\n", m_sbits);
    m_update_settings = true;
    return;
}

void SerialComm::applySettings() 
{
    DEBUG_PRINT("%s", "Applying termio settings\n");
    DEBUG_PRINT(" c_iflag = 0x%08X\n", m_term_settings.c_iflag);
    DEBUG_PRINT(" c_oflag = 0x%08X\n", m_term_settings.c_oflag);
    DEBUG_PRINT(" c_cflag = 0x%08X\n", m_term_settings.c_cflag);
    int stat = tcsetattr(m_fd, TCSANOW, &m_term_settings);
    checkAndThrow(stat, "Failed apply termios settings");
    tcflush(m_fd, TCIOFLUSH);

    m_update_settings = false;
    return;
}

void SerialComm::enableRtsCts()
{
    m_term_settings.c_cflag |= CRTSCTS;
    DEBUG_PRINT("%s\n", "RTS/CTS hardware flow control enabled");
    m_update_settings = true;
    return;
}

void SerialComm::disableRtsCts()
{
    m_term_settings.c_cflag &= ~CRTSCTS;
    DEBUG_PRINT("%s\n", "RTS/CTS hardware flow control disabled");
    m_update_settings = true;
    return;
}

void SerialComm::enableDtrDsr()
{
    // TODO: Not defined on linux...
    //m_term_settings.c_cflag |= (CDTR_IFLOW | CDSR_OFLOW);
    DEBUG_PRINT("%s\n", "DTR/DSR hardware flow control enabled");
    //m_update_settings = true;
    return;
}

void SerialComm::disableDtrDsr()
{
    // TODO: Not defined on linux...
    //m_term_settings.c_cflag &= ~(CDTR_IFLOW | CDSR_OFLOW);
    DEBUG_PRINT("%s\n", "DTR/DSR hardware flow control disabled");
    //m_update_settings = true;
    return;
}

void SerialComm::enableXOnXOff(char t_xon, char t_xoff)
{
    m_term_settings.c_iflag |= (IXON | IXOFF);
    m_term_settings.c_cc[VSTART] = t_xon;
    m_term_settings.c_cc[VSTOP] = t_xoff;
    DEBUG_PRINT("Enabling XONXOFF flow control with XON = 0x%02X, "
        "XOFF = 0x%02X\n", t_xon, t_xoff);
    m_update_settings = true;
    return;
}

void SerialComm::disableXOnXOff()
{
    m_term_settings.c_iflag &= ~(IXON | IXOFF);
    DEBUG_PRINT("%s\n", "Disabling XON/XOFF flow control");
    m_update_settings = true;
    return;
}

void SerialComm::setDtr() 
{
    int flag = TIOCM_DTR;
    int stat = ioctl(m_fd, TIOCMBIS, &flag);
    DEBUG_PRINT("%s\n", "Setting DTR");
    checkAndThrow(stat, "Failed to set DTR");
    return;
}

void SerialComm::clearDtr() 
{
    int flag = TIOCM_DTR;
    int stat = ioctl(m_fd, TIOCMBIC, &flag);
    DEBUG_PRINT("%s\n", "Clearing DTR");
    checkAndThrow(stat, "Failed to clear DTR");
    return;
}

void SerialComm::setRts() 
{
    int flag = TIOCM_RTS;
    int stat = ioctl(m_fd, TIOCMBIS, &flag);
    DEBUG_PRINT("%s\n", "Setting RTS");
    checkAndThrow(stat, "Failed to set RTS");
    return;
}

void SerialComm::clearRts() 
{
    int flag = TIOCM_RTS;
    int stat = ioctl(m_fd, TIOCMBIC, &flag);
    DEBUG_PRINT("%s\n", "Clearing RTS");
    checkAndThrow(stat, "Failed to clear RTS");
    return;
}

/*
 *      P R O T E C T E D   M E T H O D S
 */

int SerialComm::cSizeToInt(CharSize t_csize)
{
    switch (t_csize)
    {
        case CHAR_5: return 5;
        case CHAR_6: return 6;
        case CHAR_7: return 7;
        case CHAR_8: return 8;
    }
    return -1;  // never reached
}
std::string SerialComm::parToStr(Parity t_par)
{
    switch (t_par) 
    {
        case PAR_NONE: return "NONE";
        case PAR_EVEN: return "EVEN";
        case PAR_ODD: return "ODD"; 
    }   
    return "";  // never reached
}
char SerialComm::parToChar(Parity t_par)
{
    switch (t_par) 
    {
        case PAR_NONE: return 'N';
        case PAR_EVEN: return 'E';
        case PAR_ODD: return 'O';
    }
    return '\0';    // never reached
}

/*
 *      P R I V A T E   M E T H O D S
 */

void SerialComm::checkAndThrow(int t_status, const string &t_msg) const 
{
    if (t_status < 0) {
        int error = errno;
        stringstream err_msg;
        err_msg << t_msg << " (" << strerror(error) << ", " << error << ")";
        DEBUG_PRINT("%s\n", err_msg.str().c_str());

        switch (error) {
        case EAGAIN:
            throw Timeout(err_msg.str().c_str(), error);
            break;

        case ENXIO:
            throw BadConnection(err_msg.str().c_str(), error);
            break;

        default:
            throw BadIo(err_msg.str().c_str(), error);
        }
    }
    return;
}

}