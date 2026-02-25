#ifndef LK_MODBUS_HH
#define LK_MODBUS_HH

#include <memory>
#include <labkit/comms/basiccomm.hh>

namespace labkit
{

/** \brief Abstract base class for the MODBUS protocol
 *
 *  Basic MODBUS definitions used by MODBUS TCP and MODBUS RTC. This protocol
 *  requires a communication interface, but does not take ownership of it!
 */
class Modbus
{
public:
    Modbus() {};
    Modbus(std::weak_ptr<BasicComm> t_comm) : m_comm(t_comm) {};
    ~Modbus() {};
    
    /// Set communication interface
    void setComm(std::weak_ptr<BasicComm> t_comm) { t_comm = m_comm; }

    /// Modbus function codes
    enum FunctionCode : uint8_t 
    {
        FC01 = 0x01,    ///< Read coils
        FC02 = 0x02,    ///< Read discrete inputs
        FC03 = 0x03,    ///< Read multiple holding registers
        FC04 = 0x04,    ///< Read input registers
        FC05 = 0x05,    ///< Write single coil
        FC06 = 0x06,    ///< Write single holding register
        FC15 = 0x0F,    ///< Write multiple coils
        FC16 = 0x10     ///< Write multiple holding registers
    };

    /// Modbus error codes
    enum ErrorCode : uint8_t 
    {
        ERRC = 0x80,    ///< Function code for errors
        ERR1 = 0x01,    ///< Illegal Function
        ERR2 = 0x02,    ///< Illegal Data Address
        ERR3 = 0x03,    ///< Illegal Data Value
        ERR4 = 0x04     ///< Slave Device Failure
    };

    /// Function Code 01; read coils -> returns true = on, false = off
    virtual std::vector<bool> readCoils(uint8_t t_unit_id, uint16_t t_addr, 
        uint16_t t_len) = 0;

    /// Function Code 02; read discrete inputs
    virtual std::vector<bool> readDiscreteInputs(uint8_t t_unit_id, 
        uint16_t t_addr, uint16_t t_len) = 0;

    /// Function Code 03; read multiple holding registers
    virtual std::vector<uint16_t> readMultipleHoldingRegs(uint8_t t_unit_id, 
        uint16_t t_addr, uint16_t t_len) = 0;

    /// Function Code 04; read input registers
    virtual std::vector<uint16_t> readInputRegs(uint8_t t_unit_id, 
        uint16_t t_addr, uint16_t t_len) = 0;

    /// Function Code 05; write single coil -> on = true, off = false
    virtual void writeSingleCoil(uint8_t t_unit_id, uint16_t t_addr, bool t_ena) = 0;

    /// Function Code 06; write single holding register
    virtual void writeSingleHoldingReg(uint8_t t_unit_id, uint16_t t_addr, 
        uint16_t t_reg) = 0;

    /// Function Code 15; write multiple coils -> on = true, off = false
    virtual void writeMultipleCoils(uint8_t t_unit_id, uint16_t t_addr, 
        std::vector<bool> t_ena) = 0;

    /// Function Code 16; write multiple holding registers
    virtual void writeMultipleHoldingRegs(uint8_t t_unit_id, uint16_t t_addr, 
        std::vector<uint16_t> t_regs) = 0;

protected:
    /// Returns a pointer to the communication interface, if valid
    std::shared_ptr<BasicComm> getComm() const;

    /// Check error codes and throw corresponding exception
    static void checkAndThrow(uint8_t error);

private:
    std::weak_ptr<BasicComm> m_comm {};
};

}

#endif