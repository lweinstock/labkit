#include "labkit/comms/basiccomm.hh"
#include <labkit/protocols/scpi.hh>
#include <labkit/exceptions.hh>
#include <labkit/utils.hh>

#include <memory>
#include <unistd.h>

using namespace std;

namespace labkit 
{

void Scpi::setEse(uint8_t t_event_status)
{
    string msg = "*ESE " + to_string(t_event_status) + "\n";
    this->getComm()->write(msg);
    return;
}

uint8_t Scpi::getEse()
{
    string resp = this->getComm()->query("*ESE?\n");
    uint8_t ese = convertTo<uint8_t>(resp);
    return ese;
}

uint8_t Scpi::getEsr()
{
    string resp = this->getComm()->query("*ESR?\n");
    uint8_t esr = convertTo<uint8_t>(resp);
    return esr;
}

bool Scpi::getOpc()
{
    string resp = this->getComm()->query("*OPC?\n");
    bool opc {false};
    opc = convertTo<uint8_t>(resp);
    return opc;
}

void Scpi::waitForOpc(unsigned t_interval_ms)
{
    bool opc {false};
    while (!opc) {
        opc = this->getOpc();
        usleep(t_interval_ms * 1e3);  // To avoid excessive polling
    }
    return;
}

void Scpi::setSre(uint8_t service_request)
{
    string msg = "*SRE " + to_string(service_request) + "\n";
    this->getComm()->write(msg);
    return;
}

uint8_t Scpi::getSre()
{
    string resp = this->getComm()->query("*OPC?\n");
    uint8_t sre = convertTo<uint8_t>(resp);
    return sre;
}

uint8_t Scpi::getStb()
{
    string resp = this->getComm()->query("*OPC?\n");
    uint8_t stb = convertTo<uint8_t>(resp);
    return stb;
}

bool Scpi::tst()
{
    this->getComm()->write("*TST?\n");
    string resp {};
    while (resp.empty()) {  // Wait until self test is done
        resp = this->getComm()->read();
    }
    bool tst {false};
    tst = convertTo<uint8_t>(resp);
    return !tst;    // 0 = success, everything else means failed
}

/*
 *      P R I V A T E   M E T H O D S
 */

 shared_ptr<BasicComm> Scpi::getComm() const
 {
    if (m_comm.expired())
        throw BadConnection("Invalid communication interface for SCPI");
    return m_comm.lock();
 }

}