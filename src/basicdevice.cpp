#include <labkit/devices/basicdevice.hh>
#include <labkit/exceptions.hh>
#include <memory>

using namespace std;

namespace labkit
{

BasicDevice::~BasicDevice()
{
    if ( this->connected() )
        this->disconnect();
    return;
}

void BasicDevice::setComm(std::unique_ptr<BasicComm> t_comm)
{
    if ( this->connected() )
        throw DeviceError(m_name + " is already connected");

    if (!t_comm)
        throw BadConnection("Invalid communication interface (nullptr)");
    m_comm = std::move(t_comm);
    return;
}

void BasicDevice::disconnect()
{
    if (m_comm)
        m_comm.reset();
    return;
}

const std::string BasicDevice::getInfo() const 
{
    string info = m_name + ";";
    if (m_comm)
        info += m_comm->getInfo();
    else
        info += "not connected";
    return info;
}

/*
 *      P R O T E C T E D   M E T H O D S
 */

std::shared_ptr<BasicComm> BasicDevice::getComm() const
{
    if (!this->connected() )
        throw BadConnection(m_name + " is not connected");
    return m_comm;
}

}