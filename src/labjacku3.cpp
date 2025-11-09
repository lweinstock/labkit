#include <labkit/devices/labjack/labjacku3.hh>
#include <labkit/exceptions.hh>

#include <sstream>
#include <iomanip>
#include <cmath>

using namespace std;

namespace labkit
{

void LabJackU3::connect(std::unique_ptr<UsbComm> t_usb)
{
    t_usb->configInterface(0);
    t_usb->configEndpointOut(0x82);
    t_usb->configEndpointIn(0x01);

    this->setComm(std::move(t_usb));
    return;
}

void LabJackU3::setAnalogInMask(uint8_t t_eio_analog, uint8_t t_fio_analog)
{
    vector<uint8_t> msg;
    msg.push_back(W_EIO_ANALOG | W_FIO_ANALOG); // Write mask (EIO & FIO)
    msg.push_back(0x00);    // Reserved
    msg.push_back(0x00);    // TimerCounterConfig
    msg.push_back(0x00);    // DAC1Enable (always ignored)
    msg.push_back(t_fio_analog);
    msg.push_back(t_eio_analog);
    this->queryExtendedCommand(CONFIG_IO, msg);
    // HINT: On U3-HV FIO0-FIO3 are always analog!
    return;
}

void LabJackU3::getAnalogInMask(uint8_t &t_eio_analog, uint8_t &t_fio_analog)
{
    // Send empty message, only read response
    vector<uint8_t> msg(6, 0x00);
    auto resp = this->queryExtendedCommand(CONFIG_IO, msg);
    t_fio_analog = resp.at(10);
    t_eio_analog = resp.at(11);
    return;
}

void LabJackU3::setPinMode(DigitalIo t_pin, PinMode t_mode)
{
    // Read current IO config
    uint8_t fio {0x00}, eio {0x00};
    this->getAnalogInMask(eio, fio);

    if (t_pin >= CIO0)  // t_pin = CIO .. CIO3
    {
        if (t_mode == ANALOG)
            throw labkit::DeviceError("CIO port only supports digital I/O");
        else 
            return;
    }

    if (t_pin >= EIO0)  // t_pin = EI0 .. EIO7
    {
        if (t_mode == ANALOG)
            eio |= (1 << (t_pin - EIO0));   // Set bit
        else
            eio &= ~(1 << (t_pin - EIO0));  // Clear bit
    }
    else    // t_pin = FIO0 .. FIO7
    {
        if (t_mode == ANALOG)
            fio |= (1 << t_pin);    // Set bit
        else
            fio &= ~(1 << t_pin);   // Clear bit
    }

    //printf("Setting pin %u to %i (EIO 0x%02X,  FIO 0x%02X)\n", (unsigned)t_pin, 
    //    t_mode, eio, fio);

    this->setAnalogInMask(eio, fio);
    return;
}

void LabJackU3::setPinDirection(DigitalIo t_pin, PinDirection t_dir)
{
    uint8_t bits = static_cast<uint8_t>(t_pin) | (t_dir << 7);

    vector<uint8_t> msg;
    msg.push_back(0x00);    // echo
    msg.push_back(IoType::BIT_DIR_WRITE);
    msg.push_back(bits);
    auto resp = this->queryExtendedCommand(FEEDBACK, msg);
    return;
}

LabJackU3::PinState LabJackU3::getPinState(DigitalIo t_pin)
{
    vector<uint8_t> msg;
    msg.push_back(0x00);    // echo
    msg.push_back(IoType::BIT_STATE_READ);
    msg.push_back(static_cast<uint8_t>(t_pin));
    auto resp = this->queryExtendedCommand(FEEDBACK, msg);

    if (resp.at(9) == 0x00)
        return LOW;
    return HIGH;
}

void LabJackU3::setPinState(DigitalIo t_pin, PinState t_state)
{
    uint8_t bits = static_cast<uint8_t>(t_pin) | (t_state << 7);

    vector<uint8_t> msg;
    msg.push_back(0x00);    // echo
    msg.push_back(IoType::BIT_STATE_WRITE);
    msg.push_back(bits);
    auto resp = this->queryExtendedCommand(FEEDBACK, msg);
    return;
}

uint16_t LabJackU3::readAnalogRaw(AnalogInput t_pos_in, AnalogInput t_neg_in,
    bool t_long_settling, bool t_quick_sample)
{
    // Long settling and quick sample are encoded in pos_channel
    uint8_t pos = t_pos_in;
    uint8_t neg = t_neg_in;
    if (t_long_settling)
        pos |= (1 << 6);
    if (t_quick_sample)
        pos |= (1 << 7);

    vector<uint8_t> msg;
    msg.push_back(0x00);    // echo
    msg.push_back(IoType::AIN);
    msg.push_back(pos);
    msg.push_back(neg);
    auto resp = this->queryExtendedCommand(FEEDBACK, msg);

    // TODO: write separate method for feedback functions
    return ((resp[10] << 8) | resp[9]);
}

void LabJackU3::setupOneWire(DigitalIo t_dq, DigitalIo t_dpu, 
    OneWireOptions t_opt)
{
    m_dq = t_dq;
    m_dpu = t_dpu;
    m_one_wire_opt = t_opt;
    return;
}

void LabJackU3::setupI2C(DigitalIo t_sda, DigitalIo t_scl, uint8_t t_speed, 
    I2COptions t_opt)
{
    m_sda = t_sda;
    m_scl = t_scl;
    m_i2c_speed = t_speed;
    m_i2c_opt = t_opt;
    return;
}

vector<uint8_t> LabJackU3::queryOneWire(uint64_t t_rom, uint8_t t_rom_function,
    const vector<uint8_t> &t_data, size_t t_rbytes)
{
    vector<uint8_t> msg;
    // Body of the message
    msg.push_back(static_cast<uint8_t>(m_one_wire_opt));    // DPU options
    msg.push_back(0x00);    // Reserved
    msg.push_back(m_dq);    // Digital IO pin (0-15)
    msg.push_back(m_dpu);   // DPU pin
    msg.push_back(0x00);    // Reserved
    msg.push_back(t_rom_function);
    msg.push_back(static_cast<uint8_t>(0xFF & (t_rom >>  0))); // ROM0 (LSB)
    msg.push_back(static_cast<uint8_t>(0xFF & (t_rom >>  8))); // ROM1
    msg.push_back(static_cast<uint8_t>(0xFF & (t_rom >> 16))); // ROM2
    msg.push_back(static_cast<uint8_t>(0xFF & (t_rom >> 24))); // ROM3
    msg.push_back(static_cast<uint8_t>(0xFF & (t_rom >> 32))); // ROM4
    msg.push_back(static_cast<uint8_t>(0xFF & (t_rom >> 40))); // ROM5
    msg.push_back(static_cast<uint8_t>(0xFF & (t_rom >> 48))); // ROM6
    msg.push_back(static_cast<uint8_t>(0xFF & (t_rom >> 56))); // ROM7 (MSB)
    msg.push_back(0x00);    // Reserved
    msg.push_back(static_cast<uint8_t>(t_data.size()));
    msg.push_back(0x00);    // Reserved
    msg.push_back(static_cast<uint8_t>(t_rbytes));
    msg.insert(msg.end(), t_data.begin(), t_data.end());
    msg.resize(58, 0x00);   // Message has to be 64 bytes long (zero padded)

    // LabJack one wire firmware expects exactly 64 to be read! 
    auto resp = this->queryExtendedCommand(ONE_WIRE, msg, 64);

    // Remove one wire header (16 bytes)
    return vector<uint8_t>(resp.begin() + 16, resp.begin() + 16 + t_rbytes);
}

vector<uint8_t> LabJackU3::queryI2C(uint8_t t_addr, 
    const std::vector<uint8_t> &t_data, size_t t_rbytes)
{
    vector<uint8_t> msg;
    msg.push_back(static_cast<uint8_t>(m_i2c_opt)); // I2C options: bit 1 = reset at start
    msg.push_back(m_i2c_speed); // Speed adjust: 0x00 = 150kHz, 0xFF = ~10kHz
    msg.push_back(static_cast<uint8_t>(m_sda));
    msg.push_back(static_cast<uint8_t>(m_scl));
    msg.push_back(t_addr << 1); // lowest bit is set by LabJack
    msg.push_back(0x00);    // Reserved
    msg.push_back(static_cast<uint8_t>(t_data.size()));
    msg.push_back(static_cast<uint8_t>(t_rbytes));
    msg.insert(msg.end(), t_data.begin(), t_data.end());
    
    auto resp = this->queryExtendedCommand(I2C, msg);   
    // Remove I2C header (12 bytes)
    return vector<uint8_t>(resp.begin() + 12, resp.begin() + 12 + t_rbytes);
}

string LabJackU3::errorToString(uint8_t t_error_code)
{
    switch (t_error_code)
    {
        case 0x01: return "SCRATCH_WRT_FAIL";
        case 0x02: return "SCRATCH_ERASE_FAIL";
        case 0x03: return "DATA_BUFFER_OVERFLOW";
        case 0x04: return "ADC0_BUFFER_OVERFLOW";
        case 0x05: return "FUNCTION_INVALID";
        case 0x06: return "SWDT_TIME_INVALID";
        case 0x07: return "XBR_CONFIG_ERROR";
        case 0x10: return "FLASH_WRITE_FAIL";
        case 0x11: return "FLASH_ERASE_FAIL";
        case 0x12: return "FLASH_JMP_FAIL";
        case 0x13: return "FLASH_PSP_TIMEOUT";
        case 0x14: return "FLASH_ABORT_RECIEVED";
        case 0x15: return "FLASH_PAGE_MISMATCH";
        case 0x16: return "FLASH_BLOCK_MISMATCH";
        case 0x17: return "FLASH_PAGE_NOT_IN_CODE_AREA";
        case 0x18: return "MEM_ILLEGAL_ADDRESS";
        case 0x19: return "FLASH_LOCKED";
        case 0x1A: return "INVALID_BLOCK";
        case 0x1B: return "FLASH_ILLEGAL_PAGE";
        case 0x1C: return "FLASH_TOO_MANY_BYTES";
        case 0x1D: return "FLASH_INVALID_STRING_NUM";
        case 0x20: return "SMBUS_INQ_OVERFLOW";
        case 0x21: return "SMBUS_OUTQ_UNDERFLOW";
        case 0x22: return "SMBUS_CRC_FAILED";
        case 0x28: return "SHT1x_COMM_TIME_OUT";
        case 0x29: return "SHT1x_NO_ACK";
        case 0x2A: return "SHT1x_CRC_FAILED";
        case 0x2B: return "SHT1X_TOO_MANY_W_BYTES";
        case 0x2C: return "SHT1X_TOO_MANY_R_BYTES";
        case 0x2D: return "SHT1X_INVALID_MODE";
        case 0x2E: return "SHT1X_INVALID_LINE";
        case 0x30: return "STREAM_IS_ACTIVE";
        case 0x31: return "STREAM_TABLE_INVALID";
        case 0x32: return "STREAM_CONFIG_INVALID";
        case 0x33: return "STREAM_BAD_TRIGGER_SOURCE";
        case 0x34: return "STREAM_NOT_RUNNING";
        case 0x35: return "STREAM_INVALID_TRIGGER";
        case 0x36: return "STREAM_ADC0_BUFFER_OVERFLOW";
        case 0x37: return "STREAM_SCAN_OVERLAP";
        case 0x38: return "STREAM_SAMPLE_NUM_INVALID";
        case 0x39: return "STREAM_BIPOLAR_GAIN_INVALID";
        case 0x3A: return "STREAM_SCAN_RATE_INVALID";
        case 0x3B: return "STREAM_AUTORECOVER_ACTIVE";
        case 0x3C: return "STREAM_AUTORECOVER_REPORT";
        case 0x3D: return "STREAM_SOFTPWM_ON";
        case 0x3F: return "STREAM_INVALID_RESOLUTION";
        case 0x40: return "PCA_INVALID_MODE";
        case 0x41: return "PCA_QUADRATURE_AB_ERROR";
        case 0x42: return "PCA_QUAD_PULSE_SEQUENCE";
        case 0x43: return "PCA_BAD_CLOCK_SOURCE";
        case 0x44: return "PCA_STREAM_ACTIVE";
        case 0x45: return "PCA_PWMSTOP_MODULE_ERROR";
        case 0x46: return "PCA_SEQUENCE_ERROR";
        case 0x47: return "PCA_LINE_SEQUENCE_ERROR";
        case 0x48: return "TMR_SHARING_ERROR";
        case 0x50: return "EXT_OSC_NOT_STABLE";
        case 0x51: return "INVALID_POWER_SETTING";
        case 0x52: return "PLL_NOT_LOCKED";
        case 0x60: return "INVALID_PIN";
        case 0x61: return "PIN_CONFIGURED_FOR_ANALOG";
        case 0x62: return "PIN_CONFIGURED_FOR_DIGITAL";
        case 0x63: return "IOTYPE_SYNCH_ERROR";
        case 0x64: return "INVALID_OFFSET";
        case 0x65: return "IOTYPE_NOT_VALID";
        case 0x66: return "INVALID_CODE";
        case 0x70: return "UART_TIMEOUT";
        case 0x71: return "UART_NOTCONNECTED";
        case 0x72: return "UART_NOTENALBED";
        case 0x74: return "I2C_BUS_BUSY";
        case 0x76: return "TOO_MANY_BYTES";
        case 0x77: return "TOO_FEW_BYTES";
        case 0x80: return "DSP_PERIOD_DETECTION_ERROR";
        case 0x81: return "DSP_SIGNAL_OUT_OF_RANGE";
        case 0x90: return "MODBUS_RSP_OVERFLOW";
        case 0x91: return "MODBUS_CMD_OVERFLOW";
        default: return "Uknown error";
    }
    return "";
}

/*
 *  P R I V A T E   M E T H O D S 
 */

vector<uint8_t> LabJackU3::queryExtendedCommand(ExtendedCommand t_ecmd, 
    const vector<uint8_t> &t_data, size_t t_max_len)
{
    // Create header
    vector<uint8_t> msg(6, 0x00);
    msg[1] = EXT_CMD;   // Command for extended commands
    msg[2] = static_cast<uint8_t>(ceil(t_data.size()/2.)); // No of 16-bit words (always round up)
    msg[3] = t_ecmd;    // Extended command

    // Calculate checksums
    uint16_t checksum16 = this->calcChecksum16(t_data);
    msg[4] = static_cast<uint8_t>(checksum16 & 0xFF);
    msg[5] = static_cast<uint8_t>((checksum16 >> 8) & 0xFF);
    msg[0] = calcChecksum8(msg);

    // Append data
    msg.insert(msg.end(), t_data.begin(), t_data.end());
    // Data array has to be an even number (0x00 padding)
    if (msg.size() % 2)
        msg.push_back(0x00);
    if (msg.size() > EXT_CMD_MAX_LEN)
        throw labkit::BadProtocol("Message exceeds extended command length");

    auto resp = this->getComm()->queryByte(msg);

    // Parse response
    if (resp.size() == 2)
    {
        if (resp[0] == 0xB8)
            throw labkit::BadProtocol("Bad checksum");
        else if (resp[0] == 0xC8)
            throw labkit::BadProtocol("Bad extended command byte");
        else
            throw labkit::BadProtocol("Unknown error");
    }

    uint8_t error_code = resp[6];
    if (error_code != 0x00)
    {
        stringstream err("");
        err << "Received error '" << errorToString(error_code) << "'";
        err << " (0x)" << hex << setw(2) << setfill('0') << error_code << ")";
        throw labkit::BadProtocol(err.str());
    }

    return resp;
}

uint8_t LabJackU3::calcChecksum8(const std::vector<uint8_t> &t_buf)
{
    int checksum = 0;
    for (auto b : t_buf)
        checksum += b;
    int temp = checksum >> 8;
    checksum = ( checksum - 256 * temp ) + temp;
    temp = checksum/256;
    
    return (uint8_t)( ( checksum - 256 * temp ) + temp );
}

uint16_t LabJackU3::calcChecksum16(const std::vector<uint8_t> &t_buf)
{
    uint16_t checksum = 0;
    for (auto b : t_buf)
        checksum += b;
    return checksum;
}


}