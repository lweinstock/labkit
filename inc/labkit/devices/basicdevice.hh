#ifndef LK_BASIC_DEV_HH
#define LK_BASIC_DEV_HH

#include <labkit/comms/basiccomm.hh>
#include <memory>

namespace labkit
{

/**
 * @brief Virtual base class for lab devices/instruments
 * 
 */
class BasicDevice
{
public:
    virtual ~BasicDevice();

    /// No copy constructor or assignment
    BasicDevice(const BasicDevice&) = delete;
    BasicDevice& operator=(const BasicDevice&) = delete;

    /// Disconnect current connection
    virtual void disconnect();

    /// Returns true, if the device is connected and ready for communication
    bool connected() const { return (m_comm ? m_comm->good() : false); }

    /// Returns number of channels of the device
    unsigned getNChannels() const { return m_channels; }

    /// Returns a human-readable string containting device name and connection
    const std::string getInfo() const;

protected:
    /// Default ctor only for derived classes
    BasicDevice() {};
    BasicDevice(std::string t_name, unsigned t_channels = 1) 
      : m_name(t_name), m_channels(t_channels) {};

    /// Returns true if channel is valid (NOTE: 1-indexed!)
    virtual bool channelValid(unsigned t_channel) 
        { return t_channel && (t_channel <= m_channels); }

    /// Establish connection using provided unique pointer and take ownership
    void setComm(std::unique_ptr<BasicComm> t_comm);

    /// Returns a pointer to the current communication interface
    std::shared_ptr<BasicComm> getComm() const;

    /// Name of device/instrument
    std::string m_name {"Unknown device"};

private:
    std::shared_ptr<BasicComm> m_comm {nullptr};
    unsigned m_channels{1};
};

}

#endif