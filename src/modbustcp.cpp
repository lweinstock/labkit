#include <labkit/protocols/modbustcp.hh>
#include <labkit/exceptions.hh>
#include <labkit/debug.hh>

using namespace std;

namespace labkit
{

vector<bool> ModbusTcp::readCoils(uint8_t t_unit_id, uint16_t t_addr, 
    uint16_t t_len)
{
    vector<bool> ret;
    // TODO -> need device that actually uses this..
    return ret;
}

vector<bool> ModbusTcp::readDiscreteInputs(uint8_t t_unit_id, 
    uint16_t t_addr, uint16_t t_len)
{
    vector<bool> ret;
    // TODO -> need device that actually uses this..
    return ret;
}

vector<uint16_t> ModbusTcp::readMultipleHoldingRegs(uint8_t t_unit_id, 
    uint16_t t_addr, uint16_t t_len)
{
    return this->read16BitRegs(t_unit_id, FC03, t_addr, t_len);
}

vector<uint16_t> ModbusTcp::readInputRegs(uint8_t unit_id, 
    uint16_t addr, uint16_t len)
{
    return this->read16BitRegs(unit_id, FC04, addr, len);
}

void ModbusTcp::writeSingleCoil(uint8_t t_unit_id, uint16_t t_addr, bool t_ena)   
{
    // TODO -> need device that actually uses this..
    return;
}

void ModbusTcp::writeSingleHoldingReg(uint8_t t_unit_id, uint16_t t_addr, 
    uint16_t t_reg)
{
    vector<uint8_t> data {};
    data.push_back(static_cast<uint8_t>(0xFF & (t_addr >> 8)));
    data.push_back(static_cast<uint8_t>(0xFF & t_addr));
    data.push_back(static_cast<uint8_t>(0xFF & (t_reg >> 8)));
    data.push_back(static_cast<uint8_t>(0xFF & t_reg));
    auto packet = this->createPacket(t_unit_id, FC06, data);

    DEBUG_PRINT("Writing 0x%04X to address 0x%04X (tid=%u, unit_id=%u)\n",
        t_reg, t_addr, m_tid, t_unit_id);

    vector<uint8_t> resp = m_comm->queryByte(packet);

    // Extract header and data from response
    uint8_t received_fcode {0x00}, received_err {0x00};
    received_fcode = resp.at(7);
    received_err = resp.at(8);

    // Check for errors (TODO: also check received address, register and unit_id)
    if (received_fcode & ERRC)
        Modbus::checkAndThrow(received_err);
    
    // Increase transaction ID after each transaction
    m_tid++;

    return;
}

void ModbusTcp::writeMultipleCoils(uint8_t t_unit_id, uint16_t t_addr, 
    vector<bool> t_ena)
{
    // TODO -> need device that actually uses this..
    return;
}

void ModbusTcp::writeMultipleHoldingRegs(uint8_t t_unit_id, uint16_t t_addr, 
    vector<uint16_t> t_regs)
{
    uint16_t len = t_regs.size();
    vector<uint8_t> data;
    data.push_back(static_cast<uint8_t>(0xFF & (t_addr >> 8)));   // Starting register
    data.push_back(static_cast<uint8_t>(0xFF & t_addr));          // address
    data.push_back(static_cast<uint8_t>(0xFF & (len >> 8)));    // Number of registers
    data.push_back(static_cast<uint8_t>(0xFF & len));
    data.push_back(static_cast<uint8_t>(0xFF & 2*len));         // Number of bytes
    for (unsigned i = 0; i < len; i++) {
        data.push_back(static_cast<uint8_t>(0xFF & (data.at(i) >> 8)));
        data.push_back(static_cast<uint8_t>(0xFF & data.at(i)));
    }
    auto packet = this->createPacket(t_unit_id, FC16, data);

    DEBUG_PRINT("Writing %u registers with starting address 0x%04X "
        "(tid=%u, unit_id=%u)\n", len, t_addr, m_tid, t_unit_id);

    vector<uint8_t> resp = m_comm->queryByte(packet);

    // Extract header and data from response
    uint8_t received_fcode {0x00}, received_err {0x00};
    received_fcode = resp.at(7);
    received_err = resp.at(8);
    
    // Check for errors (TODO: check received address, register and unit_id)
    if (received_fcode & ERRC)
        Modbus::checkAndThrow(received_err);
    
    // Increase transaction ID after each transaction
    m_tid++;
    return;
}

/*
 *  P R I V A T E   M E T H O D S
 */

vector<uint8_t> ModbusTcp::createPacket(uint8_t t_unit_id, 
    uint8_t t_function_code, vector<uint8_t> &t_data)
{
    vector<uint8_t> packet {};

    // MODBUS TCP: Add MBAP header
    uint16_t length = static_cast<uint16_t>(2 + t_data.size());
    packet.push_back(static_cast<uint8_t>(0xFF & (m_tid >> 8)));
    packet.push_back(static_cast<uint8_t>(0xFF & m_tid));
    packet.push_back(static_cast<uint8_t>(0x00));   // Protocol ID, always
    packet.push_back(static_cast<uint8_t>(0x00));   // 0x0000
    packet.push_back(static_cast<uint8_t>(0xFF & (length >> 8)));
    packet.push_back(static_cast<uint8_t>(0xFF & length));

    // Add Protocol Data Unit (PDU)
    packet.push_back(t_unit_id);
    packet.push_back(t_function_code);
    packet.insert(packet.end(), t_data.begin(), t_data.end());

    return packet;
}

vector<uint16_t> ModbusTcp::read16BitRegs(uint8_t t_unit_id, 
    uint8_t t_function_code, uint16_t t_start_addr, uint16_t t_len)
{
    // Create packet with payload
    vector<uint8_t> data {};
    data.push_back(static_cast<uint8_t>(0xFF & (t_start_addr >> 8)));
    data.push_back(static_cast<uint8_t>(0xFF & t_start_addr));
    data.push_back(static_cast<uint8_t>(0xFF & (t_len >> 8)));
    data.push_back(static_cast<uint8_t>(0xFF & t_len));
    vector<uint8_t> packet = this->createPacket(t_unit_id, t_function_code, data);

    DEBUG_PRINT("Reading %u registers with starting address 0x%04X "
        "(tid=%u, unit_id=%u)\n", t_len, t_start_addr, m_tid, t_unit_id);

    auto resp = m_comm->queryByte(packet);

    // Extract header and data from response
    vector<uint8_t> received_data {};
    uint8_t received_fcode {0x00}, received_bytes {0x00};
    received_fcode = resp.at(7);
    received_bytes = resp.at(8);
    received_data.insert(received_data.begin(), resp.begin() + 9, resp.end());

    // Check for errors (TODO: check received bytes and unit_id)
    if (received_fcode & ERRC)
        Modbus::checkAndThrow(received_bytes);

    // Create 16-bit return vector
    vector<uint16_t> ret;
    if (received_data.size() % 2) // 0-padding if number of bytes is not even
        received_data.push_back(0x00);
    for (unsigned i = 0; i < t_len; i++)
        ret.push_back((received_data.at(2*i) << 8) | received_data.at(2*i+1));

    // MODBUS TCP: increase transaction id counter
    m_tid++;

    return ret;
}
    
}