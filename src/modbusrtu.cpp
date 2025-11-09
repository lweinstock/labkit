#include <labkit/protocols/modbusrtu.hh>
#include <labkit/debug.hh>

using namespace std;

namespace labkit
{

std::vector<bool> ModbusRtu::readCoils(uint8_t unit_id, uint16_t addr, 
    uint16_t len)
{
    vector<bool> ret;
    // TODO -> need device that actually uses this..
    return ret;
}

std::vector<bool> ModbusRtu::readDiscreteInputs(uint8_t t_unit_id, 
    uint16_t t_addr, uint16_t t_len)
{
    vector<bool> ret;
    // TODO -> need device that actually uses this..
    return ret;
}

std::vector<uint16_t> ModbusRtu::readMultipleHoldingRegs(uint8_t t_unit_id, 
    uint16_t t_addr, uint16_t t_len)
{
    return this->read16BitRegs(t_unit_id, FC03, t_addr, t_len);
}

std::vector<uint16_t> ModbusRtu::readInputRegs(uint8_t t_unit_id, 
    uint16_t t_addr, uint16_t t_len)
{
    return this->read16BitRegs(t_unit_id, FC04, t_addr, t_len);
}

void ModbusRtu::writeSingleCoil(uint8_t t_unit_id, uint16_t t_addr, bool t_ena)   
{
    // TODO -> need device that actually uses this..
    return;
}

void ModbusRtu::writeSingleHoldingReg(uint8_t t_unit_id, uint16_t t_addr, 
    uint16_t t_reg)
{
    vector<uint8_t> data {};
    data.push_back(static_cast<uint8_t>(0xFF & (t_addr >> 8)));
    data.push_back(static_cast<uint8_t>(0xFF & t_addr));
    data.push_back(static_cast<uint8_t>(0xFF & (t_reg >> 8)));
    data.push_back(static_cast<uint8_t>(0xFF & t_reg));
    auto packet = this->createPacket(t_unit_id, FC06, data);

    DEBUG_PRINT("Writing 0x%04X to address 0x%04X (unit_id=%u)\n",
        t_reg, t_addr, t_unit_id);

    vector<uint8_t> resp = m_comm->queryByte(packet);

    // Extract header and data from response
    uint8_t received_fcode {0x00}, received_err {0x00};
    received_fcode = resp.at(1);
    received_err = resp.at(2);

    // Check for errors (TODO: also check received address, register and unit_id)
    if (received_fcode & ERRC)
        Modbus::checkAndThrow(received_err);

    return;
}

void ModbusRtu::writeMultipleCoils(uint8_t t_unit_id, uint16_t t_addr, 
    std::vector<bool> t_ena)
{
    // TODO -> need device that actually uses this..
    return;
}

void ModbusRtu::writeMultipleHoldingRegs(uint8_t t_unit_id, uint16_t t_addr, 
    std::vector<uint16_t> t_regs)
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
        "(unit_id=%u)\n", len, t_addr, t_unit_id);

    vector<uint8_t> resp = m_comm->queryByte(packet);

    // Extract header and data from response
    uint8_t received_fcode = resp.at(1);
    uint8_t received_err = resp.at(2);
    
    // Check for errors (TODO: check received address, register and unit_id)
    if (received_fcode & ERRC)
        Modbus::checkAndThrow(received_err);

    return;
}

/*
 *  P R I V A T E   M E T H O D S
 */

uint16_t ModbusRtu::calcCrc16(const vector<uint8_t> &t_data)
{
    uint16_t crc = 0xFFFF;  // Start value
    for (auto pos : t_data) {
        crc ^= static_cast<uint16_t>(pos);  // XOR byte into least sig. byte of crc
        for (int i = 8; i != 0; i--) {      // Loop over each bit
            if ( (crc & 0x0001) != 0) {     // If the LSB is set
                crc >>= 1;                  // Shift right and XOR 0xA001
                crc ^= 0xA001;
            } else {                        // Else LSB is not set
                crc >>= 1;                  // Just shift right
            }
        }
    }
    // Note: this number has low and high bytes swapped, so use it accordingly 
    // (or swap bytes)
    return crc;
}

vector<uint8_t> ModbusRtu::createPacket(uint8_t t_unit_id, 
    uint8_t t_function_code, const vector<uint8_t> &t_data)
{
    vector<uint8_t> packet {};

    // Add Protocol Data Unit (PDU)
    packet.push_back(t_unit_id);
    packet.push_back(t_function_code);
    packet.insert(packet.end(), t_data.begin(), t_data.end());

    // MODBUS RTU: Append CRC checksum
    if (m_comm->type() == SERIAL) {
        uint16_t crc = this->calcCrc16(packet);
        packet.push_back(static_cast<uint8_t>(0xFF & crc));
        packet.push_back(static_cast<uint8_t>(0xFF & (crc >> 8)));
    }

    return packet;
}

vector<uint16_t> ModbusRtu::read16BitRegs(uint8_t t_unit_id, 
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
        "(unit_id=%u)\n", t_len, t_start_addr, t_unit_id);

    auto resp = m_comm->queryByte(packet);

    // Extract header and data from response
    vector<uint8_t> received_data {};
    uint8_t received_fcode = resp.at(1);
    uint8_t received_bytes = resp.at(2);
    received_data.insert(received_data.begin(), resp.begin() + 3, resp.end() - 2);

    // Check for errors (TODO: check received bytes and unit_id)
    if (received_fcode & ERRC)
        Modbus::checkAndThrow(received_bytes);

    // Create 16-bit return vector
    vector<uint16_t> ret;
    if (received_data.size() % 2) // 0-padding if number of bytes is not even
        received_data.push_back(0x00);
    for (unsigned i = 0; i < t_len; i++)
        ret.push_back((received_data.at(2*i) << 8) | received_data.at(2*i+1));

    return ret;
}

}