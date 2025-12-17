#ifndef LK_LABJACK_U3
#define LK_LABJACK_U3

#include <labkit/devices/basicdevice.hh>
#include <labkit/comms/usbcomm.hh>

namespace labkit
{

/**
 * @brief Class for the LabJack U3-LV/HV USB DAQ system
 * 
 *  Hardware documentation: https://support.labjack.com/docs/u3-datasheet
 *  USB protocol: https://support.labjack.com/docs/general-protocol-ud-devices-only
 */
class LabJackU3 : public BasicDevice
{
public:
    LabJackU3() : BasicDevice("Labjack,U3") {};
    LabJackU3(std::unique_ptr<UsbComm> t_usb) : BasicDevice()
        { this->connect(std::move(t_usb)); }
    ~LabJackU3() {};

    /// LabJack U3 Vendor ID
    static constexpr uint16_t VID = 0x0CD5;
    /// LabJack U3 Product ID
    static constexpr uint16_t PID = 0x0003;

    /// Connect to LabJack U3
    void connect(std::unique_ptr<UsbComm> t_usb);

    /* LabJack U3 commands */

    /// Analog input pin definitions 
    // TODO: Conversion from DigitalIo::FIO0 to AnalogInput::AIN0 etc. !!!
    enum AnalogInput : unsigned { AIN0 = 0, AIN1, AIN2, AIN3, AIN4, AIN5, AIN6, 
        AIN7, AIN8, AIN9, AIN10, AIN11, AIN12, AIN13, AIN14, AIN15, TEMP = 30, 
        VREG, VREF = 30, GND,
        // Alternative numbering AIN -> FIO/EIO
        FIO0_A = 0, FIO1_A, FIO2_A, FIO3_A, FIO4_A, FIO5_A, FIO6_A, FIO7_A,
        EIO0_A = 8, EIO1_A, EIO2_A, EIO3_A, EIO4_A, EIO5_A, EIO6_A, EIO7_A
    };

    /// Digital input/output pin definitions
    enum DigitalIo : unsigned { DIO0 = 0, DIO1, DIO2, DIO3, DIO4, DIO5, DIO6, 
        DIO7, DIO8, DIO9, DIO10, DIO11, DIO12, DIO13, DIO14, DIO15, DIO16, 
        DIO17, DIO18, DIO19,
        // Alternative numbering DIO -> FIO/EIO/CIO
        FIO0 = 0, FIO1, FIO2, FIO3, FIO4, FIO5, FIO6, FIO7,
        EIO0 = 8, EIO1, EIO2, EIO3, EIO4, EIO5, EIO6, EIO7,
        CIO0 = 16, CIO1, CIO2, CIO3 };

    enum PinMode : bool { DIGITAL = false, ANALOG = true };
    enum PinState: bool { LOW = false, HIGH = true };
    enum PinDirection: bool { INPUT = false, OUTPUT = true };
        
    /// One wire options for dynamic pull up (DPU)
    enum OneWireOptions : uint8_t { DPU_DISABLE = 0x00, DPU_ENABLE = 0x01, 
        DPU_POLARITY_HI = 0x02, DPU_IDLE_HI = 0x04 };

    /// I2C options
    enum I2COptions : uint8_t { NONE = 0x00, RESET_AT_START = 0x02, 
        NO_STOP_AT_RESTART = 0x04, ENABLE_CLOCK_STRETCH = 0x08 };

    /// Changes to mode of given pin to digital I/O or analog input
    void setPinMode(DigitalIo t_pin, PinMode t_mode);

    /// Set direction (input/output) of digital pin
    void setPinDirection(DigitalIo t_pin, PinDirection t_dir);

    /// Read digital state of pin (high/low)
    PinState getPinState(DigitalIo t_pin);

    /// Set digital state of pin (high/low)
    void setPinState(DigitalIo t_pin, PinState t_state);

    /// Returns raw ADC value from given analog input pin
    uint16_t readAnalogRaw(AnalogInput t_pos_in, AnalogInput t_neg_in = GND,
        bool t_long_settling = false, bool t_quick_sample = false);

    void setupOneWire(DigitalIo t_dq, DigitalIo t_dpu = DIO0, 
        OneWireOptions t_opt = DPU_DISABLE);
    
    void setupI2C(DigitalIo t_sda, DigitalIo t_scl, uint8_t t_speed = 0x00, 
        I2COptions t_opt = NONE);

    std::vector<uint8_t> queryOneWire(uint64_t t_rom, uint8_t t_rom_function,
        const std::vector<uint8_t> &t_data, size_t t_rbytes = 0);

    /// Sends a global query (0x33) to all 1-wire devices, returns ROM address
    uint64_t getRomOneWire();

    std::vector<uint8_t> queryI2C(uint8_t t_addr,
        const std::vector<uint8_t> &t_data, size_t t_rbytes = 0);

    static std::string errorToString(uint8_t t_error_code);

private:
    DigitalIo m_dq {DIO0}, m_dpu {DIO0}, m_sda {DIO0}, m_scl {DIO0};
    OneWireOptions m_one_wire_opt {DPU_DISABLE};
    I2COptions m_i2c_opt {NONE};
    uint8_t m_i2c_speed {0x00};

    static constexpr size_t FEEDBACK_MAX_LEN = 64;
    static constexpr size_t EXT_CMD_MAX_LEN = 256;

    /// Command bytes required for the normal command header
    enum Command : uint8_t
    {
        STREAM_STA  = 0xA8,
        STREAM_STO  = 0xB0,
        RESET       = 0x99,
        EXT_CMD     = 0xF8
    };

    /// Extended command bytes for the extended command header
    enum ExtendedCommand : uint8_t 
    {
        FEEDBACK    = 0x00, 
        CONFIG_U3   = 0x08, 
        CONFIG_IO   = 0x0B,
        I2C         = 0x3B,
        ONE_WIRE    = 0x3C
    };

    /// Used forn ConfigIO command to configure pin i/o
    enum ConfigIoWriteMask : uint8_t
    {
        W_TIMER_COUNTER_CFG = (1 << 0),
        W_FIO_ANALOG        = (1 << 2),
        W_EIO_ANALOG        = (1 << 3)
    };

    /// IOType for feedback requests
    enum IoType : uint8_t 
    { 
        AIN = 1,
        LED = 9,
        BIT_STATE_READ = 10,
        BIT_STATE_WRITE = 11,
        BIT_DIR_WRITE = 13 
    };

    /// Set all masked pins to analog input, all other pins are digital IOs
    void setAnalogInMask(uint8_t t_eio_analog, uint8_t t_fio_analog);

    /// Get analog input mask
    void getAnalogInMask(uint8_t &t_eio_analog, uint8_t &t_fio_analog);

    /**
     * @brief Send an extended command, check for errors and return the response
     * 
     * @param t_ecmd Extended command byte
     * @param t_data Payload of the extended command
     * @param t_max_len Maximum number of bytes to read
     * @return std::vector<uint8_t> Response (without header)
     */
    std::vector<uint8_t> queryExtendedCommand(ExtendedCommand t_ecmd, 
        const std::vector<uint8_t> &t_data, size_t t_max_len = EXT_CMD_MAX_LEN);

    /// Returns 8-bit checksum required for command header
    static uint8_t calcChecksum8(const std::vector<uint8_t> &t_buf);

    /// Returns 16-bit checksum required for extended commands
    static uint16_t calcChecksum16(const std::vector<uint8_t> &t_buf);
};

}

#endif
