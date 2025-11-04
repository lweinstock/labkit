#ifndef LK_SCPI_HH
#define LK_SCPI_HH

#include <memory>
#include <labkit/comms/basiccomm.hh>

namespace labkit
{

/** \brief Implementation of Standard Commands for Programmable Instruments (SCPI)
 *
 *  Supports common commands defined in IEEE 488.2
 */
class Scpi
{
public:
    Scpi() {};
    Scpi(std::shared_ptr<BasicComm> t_comm) : m_comm(t_comm) {};
    ~Scpi() {};

    /// Set communication interface
    void setComm(std::shared_ptr<BasicComm> t_comm) { m_comm = t_comm; }

    /// Clear communication interface
    void clearComm() { if (m_comm) m_comm.reset(); }

    /// CLear Status
    void cls() { m_comm->write("*CLS\n"); }

    /// Event Status Enable command
    void setEse(uint8_t t_event_status);

    /// Event Status Enable query
    uint8_t getEse();

    /// Event Status Read
    uint8_t getEsr();

    /// Standard Event Status Register (SESR) definitions
    enum SESR : uint8_t {
        OPC = (1 << 0),     //> Operation Complete
        RQC = (1 << 1),     //> Request Control
        QYE = (1 << 2),     //> Query Error
        DDE = (1 << 3),     //> Device Dependant Error
        EXE = (1 << 4),     //> Execution Error
        CME = (1 << 5),     //> Command Error
        URQ = (1 << 6),     //> User Request
        PON = (1 << 7)      //> Power On
    };

    /// IDeNtification query
    std::string getIdn() { return m_comm->query("*IDN?\n"); }

    /// OPeration Complete command
    void setOpc() { m_comm->write("*OPC\n"); }

    /// OPeration Complete query
    bool getOpc();

    /// Wait for OPC (blocking)
    void waitForOpc(unsigned t_interval_ms = 100);

    /// ReSeT
    void rst() { m_comm->write("*RST\n"); }

    /// Service Request Enable command
    void setSre(uint8_t t_service_request);

    /// Serivce Request Enable query
    uint8_t getSre();

    /// STatus Byte query
    uint8_t getStb();

    /// Self TeST query
    bool tst();

    /// WAIt to continue
    void wai() { m_comm->write("*WAI\n"); }

private:
    std::shared_ptr<BasicComm> m_comm {nullptr};
};

}

#endif