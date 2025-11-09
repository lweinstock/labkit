#include <labkit/protocols/modbus.hh>
#include <labkit/exceptions.hh>

namespace labkit
{

void Modbus::checkAndThrow(uint8_t error)
{
    switch (error) {
    case ERR1:
        throw BadProtocol("Function code not supported");
        break;
    case ERR2:
        throw BadProtocol("Starting address or last address not supported");
        break;
    case ERR3:
        throw BadProtocol("Quantity of registers not supported (range 1 - 125)");
        break;
    case ERR4:
        throw BadProtocol("No read access to registers");
        break;
    default:
        throw BadProtocol("Unknown error code " + std::to_string(error));
    }
    return; 
}    

}