#ifndef LK_USBTMC_COMM_HH
#define LK_USBTMC_COMM_HH

#include <labkit/comms/usbcomm.hh>

namespace labkit
{

/**
 * @brief USBTMC communication interface
 * 
 *  This class provides a USB communication interface based on the USB
 *  Test and Measurement Class (TMC) protocol definitions. USBTMC defines
 *  device dependant and vendor specific message transfers using bulk endpoints.
 *
 *  USBTMC is often used by instruments utilizing the Standard Commands for
 *  Programmable Instruments (SCPI).
 */
class UsbTmcComm : public UsbComm {
public:
    /// Default constructor
    UsbTmcComm() : UsbComm() {};

    /**
     * @brief Open USBTMC communication to device with given VID, PID, and serial
     * 
     * @param t_vid Vendor ID
     * @param t_pid Product ID
     * @param t_serno Serial number
     */
    UsbTmcComm(uint16_t t_vid, uint16_t t_pid, std::string t_serno = "")
      : UsbComm(t_vid, t_pid, t_serno) {};

    /// Destructor
    ~UsbTmcComm() {};

    /** \brief C-style raw byte write
     *
     *  Performs a USBTMC device dependant message transfer.
     *
     *  \param [in] t_data Output byte array.
     *  \param [in] t_len Length of byte array.
     *  \return Number of successfully written bytes.
     */
    int writeRaw(const uint8_t* t_data, size_t t_len) override;

    /** \brief C-style raw byte read.
     *
     *  Performs a USBTMC device dependant message transfer.
     *
     *  \param [out] t_data Input byte array.
     *  \param [in] t_max_len Maximum length of byte array.
     *  \return Number of successfully read bytes.
     */
    int readRaw(uint8_t* t_data, size_t t_max_len, 
        unsigned t_timeout_ms = DFLT_TIMEOUT_MS) override;

    CommType type() const noexcept override { return USBTMC; }

    /// USBTMC device dependant data write
    int writeDevDepMsg(const uint8_t* t_msg, size_t t_len,
        uint8_t t_transfer_attr = EOM);
    /// USBTMC device dependant data read
    int readDevDepMsg(uint8_t* t_data, size_t t_max_len,
        int t_timeout_ms = DFLT_TIMEOUT_MS, uint8_t t_transfer_attr = TERM_CHAR, 
        uint8_t t_term_char = '\n');

    /// USBTMC vendor specific data write
    int writeVendorSpecific(std::string msg);
    /// USBTMC vendor specific data read
    std::string readVendorSpecific(int timeout_ms = DFLT_TIMEOUT_MS);

private:

    // USBTMC protocol definitions
    static constexpr unsigned HEADER_LEN = 12;
    static constexpr uint8_t LIBUSB_SUBCLASS_TMC = 0x03;
    enum bRequest : uint16_t {
        INITIATE_ABORT_BULK_OUT     = 0x01,
        CHECK_ABORT_BULK_OUT_STATUS = 0x02,
        INITIATE_ABORT_BULK_IN      = 0x03,
        CHECK_ABORT_BULK_IN_STATUS  = 0x04,
        INITIATE_CLEAR              = 0x05,
        CHECK_CLEAR_STATUS          = 0x06,
        GET_CAPABILITIES            = 0x07,
        INDICATOR_PULSE             = 0x40
    };

    enum MsgID : uint16_t {
        DEV_DEP_MSG_OUT             = 0x01,
        REQUEST_DEV_DEP_MSG_IN      = 0x02,
        DEV_DEP_MSG_IN              = 0x02,
        VENDOR_SPECIFIC_OUT         = 0x7E,
        REQUEST_VENDOR_SPECIFIC_IN  = 0x7F,
        VENDOR_SPECIFIC_IN          = 0x7F
    };

    enum bmTransferAttributes : uint16_t {
        EOM = 0x01,
        TERM_CHAR = 0x02
    };

    uint8_t m_cur_tag {0x01}, m_term_char {0x00};

    // Creates a USBTMC header
    std::vector<uint8_t> createUsbTmcHeader(uint8_t t_message_id, 
        uint8_t t_transfer_attr, uint32_t t_transfer_size, uint8_t t_term_char = 0x00);

    // Extract data from header, check bTag fields, returns transfer length
    int checkUsbUmcHeader(std::vector<uint8_t> t_message, uint8_t t_message_id);

};

}

#endif