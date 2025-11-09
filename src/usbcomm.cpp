#include <labkit/comms/usbcomm.hh>
#include <labkit/exceptions.hh>
#include <labkit/debug.hh>

#include <sstream>
#include <algorithm>
#include <iostream>
#include <iomanip>
#include <cstdint>
#include <cstdio>

using namespace std;

namespace labkit {

libusb_context* UsbComm::s_default_ctx = NULL;
int UsbComm::s_dev_count = 0;

UsbComm::UsbComm(uint16_t t_vid, uint16_t t_pid, string t_serno) : UsbComm() 
{
    this->open(t_vid, t_pid, t_serno);
    return;
}

UsbComm::~UsbComm()
{
    if (this->good())
        this->close();
    return;
}

int UsbComm::writeRaw(const uint8_t* t_data, size_t t_len)
{
    if (m_cur_iface == -1)
        throw BadIo(this->getInfo() + " - No USB interface claimed");

    uint8_t bmRequestType {0x00}, bRequest {0x00};
    uint16_t wValue {0x0000}, wIndex {0x0000};
    switch (m_ep_out_type)
    {
        case CONTROL:
        if (t_len < 6)
            throw BadProtocol(this->getInfo() + "Control write requires at "
                "least 6 bytes of data");
        bmRequestType = t_data[0];
        bRequest = t_data[1];
        wValue = (t_data[2] << 8) | t_data[3];
        wIndex = (t_data[4] << 8) | t_data[5];
        return this->controlTransfer(bmRequestType, bRequest, wValue, wIndex, 
            &t_data[6], t_len - 6);
        
        case BULK:
        return this->writeBulk(t_data, t_len);

        case INTERRUPT:
        return this->writeInterrupt(t_data, t_len);
        
        default:
        break;
    }
    return -1;
}

int UsbComm::readRaw(uint8_t* t_data, size_t t_max_len, unsigned t_timeout_ms)
{
    if (m_cur_iface == -1)
        throw BadIo(this->getInfo() + " - No USB interface claimed");

    uint8_t bmRequestType {0x00}, bRequest {0x00};
    uint16_t wValue {0x0000}, wIndex {0x0000};
    switch (m_ep_in_type)
    {
        case CONTROL:
        if (t_max_len < 6)
            throw BadProtocol(this->getInfo() + "Control read requires at "
                "least 6 bytes of data");
        bmRequestType = t_data[0];
        bRequest = t_data[1];
        wValue = (t_data[2] << 8) | t_data[3];
        wIndex = (t_data[4] << 8) | t_data[5];
        return this->controlTransfer(bmRequestType, bRequest, wValue, wIndex, 
            &t_data[6], t_max_len - 6, t_timeout_ms);

        case BULK:
        return this->readBulk(t_data, t_max_len, t_timeout_ms);

        case INTERRUPT:
        return this->readInterrupt(t_data, t_max_len, t_timeout_ms);
        
        default:
        break;
    }
    return -1;
}

void UsbComm::open()
{
    this->open(m_vid, m_pid, m_serno);
    return;
}

void UsbComm::open(uint16_t t_vid, uint16_t t_pid, string t_serno)
{
    int stat;
    // If first device, start new libusb session
    if (s_dev_count == 0) {
        stat = libusb_init(&s_default_ctx);
        check_and_throw(stat, "libusb init failed");
        DEBUG_PRINT("new libusb session initialized (%i)\n", stat);
    }

    // Search for deivce with given VID & PID
    libusb_device** dev_list;
    int ndev = libusb_get_device_list(s_default_ctx, &dev_list);
    check_and_throw(ndev, "Failed to get device list");

    for (int idev = 0; idev < ndev; idev++) {
        // Get dev descriptor for vid, pid, bus, and port
        libusb_device* tmp_dev = dev_list[idev];
        libusb_device_descriptor desc;
        stat = libusb_get_device_descriptor(tmp_dev, &desc);
        check_and_throw(stat, "Failed to get device descriptor");

        DEBUG_PRINT("device %i: VID:PID=0x%04X:0x%04X\n", 
            idev, desc.idVendor, desc.idProduct);

        // Check for VID and PID
        if ( (desc.idVendor == t_vid) && (desc.idProduct == t_pid) ) {
            // Also check for serial number, if specified
            if ( !t_serno.empty() ) {
                libusb_device_handle* tmp_handle;
                stat = libusb_open(tmp_dev, &tmp_handle);
                check_and_throw(stat, "Failed to get usb handle");

                uint8_t serial_cstr[126];
                stat = libusb_get_string_descriptor_ascii(tmp_handle,
                    desc.iSerialNumber, serial_cstr, 126);
                check_and_throw(stat, "Failed to get string descriptor");
                DEBUG_PRINT("SerialNumber %s\n", serial_cstr);
                string serial_str( (char*)serial_cstr );

                // If serial number does not match, skip rest
                if (t_serno != serial_str) continue;
            }
            m_usb_dev = tmp_dev;
            m_vid = t_vid;
            m_pid = t_pid;
            m_serno = t_serno;
            break;
        }
    }

    // Get I/O handle
    if (m_usb_dev) {
        stat = libusb_open(m_usb_dev, &m_usb_handle);
        check_and_throw(stat, "Failed to get usb handle");
    } else {
        stringstream msg("");
        msg << hex << uppercase << setw(4) << setfill('0');
        msg << "Device 0x" << t_vid << ":0x" << t_pid << " not found";
        throw BadConnection(msg.str());
    }
    s_dev_count++;
    DEBUG_PRINT("Opened device, new device count = %i\n", s_dev_count);

    libusb_free_device_list(dev_list, 1);
    m_good = true;
    return;
}

void UsbComm::close()
{
    // Release claimed interfaces and device
    if (m_cur_iface != -1)
        libusb_release_interface(m_usb_handle, m_cur_iface);
    if (m_usb_handle)
        libusb_close(m_usb_handle);

    // Close context if last device released
    s_dev_count--;
    DEBUG_PRINT("Closed device, new device count = %i\n", s_dev_count);
    if (s_dev_count == 0) {
        DEBUG_PRINT("%s\n", "Last device closed, exiting libusb\n");
        libusb_exit(s_default_ctx);
    }

    m_good = false;
    return;
}

string UsbComm::getInfo() const noexcept
{
    char buf[64];
    snprintf(buf, 64, "usb;%04X:%04X", m_vid, m_pid);
    string info(buf);
    if (!m_serno.empty())
        info += ":" + m_serno;
    return info;
}

void UsbComm::clear()
{
    libusb_clear_halt(m_usb_handle, m_ep_in_addr);
    libusb_clear_halt(m_usb_handle, m_ep_out_addr);
    return;
}

int UsbComm::controlTransfer(uint8_t t_request_type, uint8_t t_request, uint16_t t_value, 
    uint16_t t_index, const uint8_t* t_data, int t_len, int t_timeout_ms) 
{
    if (m_cur_iface == -1)
        throw BadIo(this->getInfo() + " - No USB interface claimed");

    int nbytes = libusb_control_transfer(
        m_usb_handle,
        t_request_type,
        t_request,
        t_value,
        t_index,
        (uint8_t*)t_data,
        t_len,
        t_timeout_ms);
    check_and_throw(nbytes, "Control transfer to endpoint 0 failed");
    DEBUG_PRINT_BYTE_DATA(t_data, nbytes, "Written %zu bytes: ", nbytes);
    return nbytes;
}

int UsbComm::writeBulk(const uint8_t* t_data, int t_len) 
{
    if (m_cur_iface == -1)
        throw BadIo(this->getInfo() + " - No USB interface claimed");

    int stat, nbytes = 0;
    size_t bytes_left = t_len;
    size_t bytes_written = 0;

    while ( bytes_left > 0 ) {
        // Packets can be max wMaxPacketSize
        stat = libusb_bulk_transfer(
            m_usb_handle,
            m_ep_out_addr,
            (uint8_t*)&t_data[bytes_written],
            min(bytes_left, m_max_pkt_size_out),
            &nbytes,
            1000);
        char msg[128];
        snprintf(msg, 128, "Bulk transfer (write) to endpoint 0x%02X failed", 
            m_ep_out_addr);
        check_and_throw(stat, string(msg));
        bytes_left -= nbytes;
        bytes_written += nbytes;
        DEBUG_PRINT_BYTE_DATA(t_data, nbytes, "Written %zu bytes: ", nbytes);
    }
    return nbytes;
}

int UsbComm::readBulk(uint8_t* t_data, int t_max_len, int t_timeout_ms) 
{
    if (m_cur_iface == -1)
        throw BadIo(this->getInfo() + " - No USB interface claimed");

    int nbytes = 0;
    int stat = libusb_bulk_transfer(
        m_usb_handle,
        m_ep_in_addr,
        t_data,
        t_max_len,
        &nbytes,
        t_timeout_ms);
    char msg[128];
    snprintf(msg, 128, "Bulk transfer (read) to endpoint 0x%02X failed", 
        m_ep_in_addr);
    check_and_throw(stat, string(msg));
    DEBUG_PRINT_BYTE_DATA(t_data, nbytes, "Read %zu bytes: ", nbytes);
    return nbytes;
}

int UsbComm::writeInterrupt(const uint8_t* t_data, int t_len) 
{
    // TODO
    return 0;
}

int UsbComm::readInterrupt(uint8_t* t_data, int t_max_len,
    int timeout_ms) 
{
    // TODO
    return 0;
}

void UsbComm::configInterface(int t_iface, int t_alt) 
{
    int stat;
    string msg("");

    // Release current interface (-1 = no interface claimed)
    if (m_cur_iface != -1)
    {
        stat = libusb_release_interface(m_usb_handle, m_cur_iface);
        msg = "Failed to release interface " + to_string(m_cur_iface);
        check_and_throw(stat, msg);
        m_cur_iface = -1;
    }

    // Detach kernel drivers if active
    if ( libusb_kernel_driver_active(m_usb_handle, t_iface) ) 
    {
        stat = libusb_detach_kernel_driver(m_usb_handle, t_iface);
        check_and_throw(stat, "Failed to detach kernel driver from interface");
    }

    // Claim new interface
    stat = libusb_claim_interface(m_usb_handle, t_iface);
    msg = "Failed to claim interface " + to_string(t_iface);
    check_and_throw(stat, msg);
    m_cur_iface = t_iface;
    DEBUG_PRINT("Successfully claimed interface %i\n", m_cur_iface);

    // Apply alternate settingsm if specified
    if (t_alt) 
    {
        stat = libusb_set_interface_alt_setting(m_usb_handle, m_cur_iface, t_alt);
        msg = "Failed to apply alternate settings " + to_string(t_alt);
        check_and_throw(stat, msg);
        DEBUG_PRINT("Applied alternate settings %i\n", t_alt);
    }
    return;
}

void UsbComm::configEndpointIn(uint8_t t_ep_addr, EndpointType t_type, 
    size_t t_max_size)
{
    if (m_cur_iface == -1)
        throw BadIo(this->getInfo() + " - No USB interface claimed");

    if (t_ep_addr < LIBUSB_ENDPOINT_IN)
        throw BadIo(this->getInfo() + " - wrong endpoint (IN) direction");

    m_ep_in_type = t_type;
    m_ep_in_addr = t_ep_addr;
    m_max_pkt_size_in = t_max_size;
    DEBUG_PRINT("Setting endpoint (IN): addr 0x%02X, wMaxPacketSize %lu\n", 
        m_ep_in_addr, m_max_pkt_size_in);

    return;
}

void UsbComm::configEndpointOut(uint8_t t_ep_addr, EndpointType t_type, 
    size_t t_max_size)
{
    if (m_cur_iface == -1)
        throw BadIo(this->getInfo() + " - No USB interface claimed");

    if (t_ep_addr >= LIBUSB_ENDPOINT_IN)
        throw BadIo(this->getInfo() + " - wrong endpoint (OUT) direction");

    m_ep_out_type = t_type;
    m_ep_out_addr = t_ep_addr;
    m_max_pkt_size_out = t_max_size;
    DEBUG_PRINT("Setting endpoint (OUT): addr 0x%02X, wMaxPacketSize %lu\n", 
        m_ep_out_addr, m_max_pkt_size_out);

    return;
}

/*
 *      P R I V A T E   M E T H O D S
 */

void UsbComm::check_and_throw(int t_stat, const string& t_msg) const 
{
    if (t_stat < 0) {
        stringstream err_msg;
        err_msg << this->getInfo() << " - " << t_msg;
        err_msg << " (" << libusb_error_name(t_stat) << ", " << t_stat << ")";
        DEBUG_PRINT("%s\n", err_msg.str().c_str());

        switch (t_stat) {
        case LIBUSB_ERROR_TIMEOUT:
        case LIBUSB_ERROR_BUSY:
            throw Timeout(err_msg.str().c_str(), t_stat);
            break;

        case LIBUSB_ERROR_OVERFLOW:
        case LIBUSB_ERROR_IO:
        case LIBUSB_ERROR_NO_MEM:
        case LIBUSB_ERROR_OTHER:
            throw BadIo(err_msg.str().c_str(), t_stat);
            break;

        case LIBUSB_ERROR_ACCESS:
        case LIBUSB_ERROR_NO_DEVICE:
            throw BadConnection(err_msg.str().c_str(), t_stat);
            break;

        case LIBUSB_ERROR_INVALID_PARAM:
        case LIBUSB_ERROR_NOT_SUPPORTED:
        default:
            fprintf(stderr, "%s\n", err_msg.str().c_str());
            abort();
        }
    }
    return;
}

}