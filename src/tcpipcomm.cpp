#include <labkit/comms/tcpipcomm.hh>
#include <labkit/exceptions.hh>
#include <labkit/debug.hh>

#include <errno.h>
#include <netinet/in.h>
#include <sys/time.h>
#include <string.h>
#include <sstream>

using namespace std;

namespace labkit 
{

TcpipComm::~TcpipComm()
{
    if (this->good())
        this->close();
    return;
}

TcpipComm::TcpipComm(std::string t_ip_addr, unsigned t_port) 
{ 
    this->open(t_ip_addr, t_port); 
    return;
}

void TcpipComm::open(std::string t_ip_addr, unsigned t_port)
{
    // Create TCP/IP socket
    m_socket_fd = socket(AF_INET, SOCK_STREAM, 0);
    checkAndThrow(m_socket_fd, "Could not open socket.");

    setBufferSize(DFLT_BUF_SIZE);
    setTimeout(0); // never time out -> we will use select() for timeout

    // Set up instrument ip address
    m_instr_addr.sin_family = AF_INET;
    m_instr_addr.sin_port = htons(t_port);
    int stat = inet_aton(t_ip_addr.c_str(), &m_instr_addr.sin_addr);
    stringstream err_msg("");
    err_msg << "Address " << t_ip_addr << ":" << t_port << " is not supported.";
    checkAndThrow(stat, err_msg.str());

    // Connect to instrument...
    stat = connect(m_socket_fd, (struct sockaddr *)&m_instr_addr,
        sizeof(m_instr_addr));
    err_msg.str("");
    err_msg << "Failed to connect to " << t_ip_addr << ":" << t_port << ".";
    checkAndThrow(stat, err_msg.str());
    DEBUG_PRINT("Connected to IP address = %s:%u\n",
        inet_ntoa(m_instr_addr.sin_addr), ntohs(m_instr_addr.sin_port));
    m_ip_addr = t_ip_addr;
    m_port = t_port;

    m_good = true;
    return;
}

void TcpipComm::close()
{
    DEBUG_PRINT("Closing connection to %s:%i\n", m_ip_addr.c_str(), m_port);
    //shutdown(m_socket_fd, SHUT_WR);
    //while ( !this->read().empty() ) { usleep(100e3); } // Read remaining messages
    //shutdown(m_socket_fd, SHUT_RD);
    shutdown(m_socket_fd, SHUT_RDWR);
    int stat = ::close(m_socket_fd);
    checkAndThrow(stat, "Failed to close connection to " + m_ip_addr + ":" 
        + to_string(m_port));

    m_good = false;
    return;
}

int TcpipComm::writeRaw(const uint8_t* t_data, size_t t_len) 
{
    size_t bytes_left = t_len;
    size_t bytes_written = 0;
    ssize_t nbytes = 0;

    while ( bytes_left > 0 ) {
        nbytes = send(m_socket_fd, &t_data[bytes_written], bytes_left, 0);
        checkAndThrow(nbytes, "Failed to write to device");
        bytes_left -= nbytes;

        DEBUG_PRINT_BYTE_DATA(t_data, nbytes, "Written %zu bytes: ", nbytes);

        bytes_written += nbytes;
    }

    return bytes_written;
}

int TcpipComm::readRaw(uint8_t* t_data, size_t t_max_len, unsigned t_timeout_ms)
{
    // Wait for I/O
    fd_set rfd_set;
    FD_ZERO(&rfd_set);
    FD_SET(m_socket_fd, &rfd_set);
    m_timeout.tv_sec = t_timeout_ms / 1000;
    m_timeout.tv_usec = t_timeout_ms % 1000;

    // Block until data is available or timeout exceeded
    int stat = select(m_socket_fd + 1, &rfd_set, NULL, NULL, &m_timeout);
    checkAndThrow(stat, "No data available");
    if (stat ==  0)
        throw Timeout(this->getInfo() + " - Read timeout occurred", errno);

    ssize_t nbytes = recv(m_socket_fd, t_data, t_max_len, 0);
    checkAndThrow(nbytes, "Failed to read from device");
    DEBUG_PRINT_BYTE_DATA(t_data, nbytes, "Read %zu bytes: ", nbytes);

    return nbytes;
}

void TcpipComm::setBufferSize(size_t t_size) 
{
    // Receive buffer
    int stat = setsockopt(m_socket_fd, SOL_SOCKET, SO_RCVBUF, (char*)&t_size,
        sizeof(int));
    stringstream err_msg("");
    err_msg << "Set receive buffer length to " << t_size << " failed.";
    checkAndThrow(stat, err_msg.str());

    // Send buffer
    stat = setsockopt(m_socket_fd, SOL_SOCKET, SO_SNDBUF, (char *)&t_size,
        sizeof(int));
    err_msg.str("");
    err_msg << "Set send buffer length to " << t_size << "failed.";
    checkAndThrow(stat, err_msg.str());

    return;
}

void TcpipComm::setTimeout(unsigned t_timeout_ms) 
{
    struct timeval timeout;
    timeout.tv_sec = t_timeout_ms / 1000;
    timeout.tv_usec = 1000 * (t_timeout_ms % 1000);

    int stat = setsockopt(m_socket_fd, SOL_SOCKET, SO_RCVTIMEO,
        (char*)&timeout, sizeof(timeout));
    stringstream err_msg("");
    err_msg << "Set receive timeout to " << t_timeout_ms << "ms failed.";
    checkAndThrow(stat, err_msg.str());

    stat = setsockopt(m_socket_fd, SOL_SOCKET, SO_SNDTIMEO,
        (char *)&timeout, sizeof(timeout));
    err_msg.str("");
    err_msg << "Set send timeout to " << t_timeout_ms << "ms failed.";
    checkAndThrow(stat, err_msg.str());

    return;
}

string TcpipComm::getInfo() const noexcept
{
    // Format example: tcpip;192.168.0.1;10001
    string ret("tcpip;" + m_ip_addr + ";" + to_string(m_port));
    return ret;
}

/*
 *      P R I V A T E   M E T H O D S
 */


void TcpipComm::checkAndThrow(int status, const string &msg) const 
{
    if (status < 0) {
        int error = errno;
        stringstream err_msg;
        err_msg << this->getInfo() << " - " << msg;
        err_msg << " (" << strerror(error) << ", " << error << ")";
        DEBUG_PRINT("%s\n", err_msg.str().c_str());

        switch (error) {
        case EAGAIN:
            throw Timeout(err_msg.str().c_str(), error);
            break;

        case ENXIO:
        case ECONNREFUSED:
            throw BadConnection(err_msg.str().c_str(), error);
            break;

        default:
            throw BadIo(err_msg.str().c_str(), error);
        }
    }
    return;
}

}