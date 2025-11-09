#include <labkit/comms/usbtmccomm.hh>
#include <labkit/exceptions.hh>
#include <labkit/debug.hh>

using namespace std;

namespace labkit 
{

int UsbTmcComm::writeRaw(const uint8_t* t_data, size_t t_len) 
{
    return this->writeDevDepMsg(t_data, t_len);
}

int UsbTmcComm::readRaw(uint8_t* t_data, size_t t_max_len, unsigned t_timeout_ms) 
{
    return this->readDevDepMsg(t_data, t_max_len, t_timeout_ms);
}

int UsbTmcComm::writeDevDepMsg(const uint8_t* t_msg, size_t t_len,
    uint8_t transfer_attr) 
{
    // Create header
    auto write_request = this->createUsbTmcHeader(DEV_DEP_MSG_OUT, 
        transfer_attr, t_len);

    // Append data
    write_request.insert(write_request.end(), t_msg, t_msg + t_len);

    // Length has to be a multiple of 4; add zero padding if necessary
    if (write_request.size() % 4)
    {
        size_t len = write_request.size() + 4 - (write_request.size() % 4);
        write_request.resize(len, 0x00);
    }

    DEBUG_PRINT("%s\n", "Sending device dependent message");
    auto wbuf = reinterpret_cast<const uint8_t*>(&write_request[0]);
    int nbytes = this->writeBulk(wbuf, write_request.size());

    return nbytes;
}

int UsbTmcComm::readDevDepMsg(uint8_t* t_data, size_t t_max_len,
    int t_timeout_ms, uint8_t t_transfer_attr, uint8_t t_term_char) 
{
    // Send read request
    auto read_request = this->createUsbTmcHeader(REQUEST_DEV_DEP_MSG_IN,
        t_transfer_attr, DFLT_BUF_SIZE, t_term_char);
    auto rqst = reinterpret_cast<const uint8_t*>(&read_request[0]);

    DEBUG_PRINT("%s\n", "Sending read request");
    this->writeBulk(rqst, read_request.size());

    // Read from bulk endpoint
    DEBUG_PRINT("%s\n", "Reading device dependent message");
    vector<uint8_t> rbuf(DFLT_BUF_SIZE);
    int len = this->readBulk(&rbuf[0], rbuf.size(), t_timeout_ms);

    // If an empty message was received, return immediatly
    if (len == 0)
        return len;

    // Check header
    int transfer_size = checkUsbUmcHeader(rbuf, DEV_DEP_MSG_IN);

    // Copy data (without header) into output array
    std::copy(rbuf.begin() + HEADER_LEN, rbuf.begin() + len, t_data);
    // Continue reading if not all data has been received yet
    int bytes_received = len - HEADER_LEN;
    while (bytes_received < transfer_size) {
        int nbytes = this->readBulk(&rbuf[0], rbuf.size(), t_timeout_ms);
        if (bytes_received > static_cast<int>(t_max_len))
            throw BadIo(this->getInfo() + " - Buffer size too small");
        std::copy(rbuf.begin(), rbuf.begin() + nbytes, t_data + bytes_received);
        bytes_received += nbytes;
    }
    DEBUG_PRINT_BYTE_DATA(t_data, bytes_received, "Read %zu bytes: ", bytes_received);

    // Increase bTag for next communication
    m_cur_tag++;
    
    return bytes_received;
}

int UsbTmcComm::writeVendorSpecific(string t_msg) 
{
    auto write_request = this->createUsbTmcHeader(VENDOR_SPECIFIC_OUT, 0x00,
        t_msg.size());

    // Append data
    write_request.insert(write_request.end(), t_msg.begin(), t_msg.end());

    // Length has to be a multiple of 4; add zero padding if necessary
    if (write_request.size() % 4)
    {
        size_t len = write_request.size() + 4 - (write_request.size() % 4);
        write_request.resize(len, 0x00);
    }

    auto wbuf = reinterpret_cast<const uint8_t*>(&write_request[0]);
    int nbytes = this->writeBulk(wbuf, write_request.size());

    return nbytes;
}

string UsbTmcComm::readVendorSpecific(int t_timeout_ms) 
{
    // Send read request
    DEBUG_PRINT("%s\n", "Sending vendor specific read request\n");
    auto read_request = this->createUsbTmcHeader(REQUEST_VENDOR_SPECIFIC_IN,
        0x00, DFLT_BUF_SIZE, 0x00);
    this->writeBulk(&read_request[0], read_request.size());

    // Read from bulk endpoint
    DEBUG_PRINT("%s\n", "Reading vendor specific message");
    vector<uint8_t> rbuf(DFLT_BUF_SIZE);
    int len = this->readBulk(&rbuf[0], rbuf.size(), t_timeout_ms);

    // If an empty message was received, return immediatly
    if (len == 0)
        return "";

    // Check header
    int transfer_size = checkUsbUmcHeader(rbuf, DEV_DEP_MSG_IN);

    // Remove header from return value
    string ret(rbuf.begin() + HEADER_LEN, rbuf.end());
    // If more data than received was anounced in the header, keep reading
    int bytes_left = transfer_size - len;
    while (bytes_left > 0) {
        int nbytes = this->readBulk(&rbuf[0], rbuf.size(), t_timeout_ms);
        ret.append(rbuf.begin(), rbuf.begin() + min(bytes_left, nbytes));
        bytes_left -= nbytes;
    }
    DEBUG_PRINT("Received vendor specific message (%lu) '%s'\n", ret.size(), 
        ret.c_str());

    // Increase bTag for next communication
    m_cur_tag++;

    return ret;
}

/*
 *      P R I V A T E   M E T H O D S
 */

vector<uint8_t> UsbTmcComm::createUsbTmcHeader(uint8_t t_message_id, 
    uint8_t t_transfer_attr, uint32_t t_transfer_size, uint8_t t_term_char) 
{
    vector<uint8_t> header(HEADER_LEN);
    // Create USBTMC t_header
    header[0] = t_message_id;
    header[1] = m_cur_tag;
    header[2] = ~m_cur_tag;
    header[3] = 0x00;
    header[4] = 0xFF & t_transfer_size;
    header[5] = 0xFF & (t_transfer_size >> 8);
    header[6] = 0xFF & (t_transfer_size >> 16);
    header[7] = 0xFF & (t_transfer_size >> 24);
    switch (t_message_id) {
        case DEV_DEP_MSG_OUT:
        header[8] = 0x01 & t_transfer_attr;
        header[9] = 0x00;
        header[10] = 0x00;
        header[11] = 0x00;
        break;

        case REQUEST_DEV_DEP_MSG_IN:
        header[8] = 0x02 & t_transfer_attr;
        header[9] = t_term_char;
        header[10] = 0x00;
        header[11] = 0x00;
        if (t_transfer_attr & TERM_CHAR)
            m_term_char = t_term_char;
        break;

        case VENDOR_SPECIFIC_OUT:
        case REQUEST_VENDOR_SPECIFIC_IN:
        header[8] = 0x00;
        header[9] = 0x00;
        header[10] = 0x00;
        header[11] = 0x00;
        break;
    }

    return header;
}

int UsbTmcComm::checkUsbUmcHeader(vector<uint8_t> t_message, uint8_t t_message_id) 
{
    if (t_message.size() < HEADER_LEN)
        throw BadProtocol("Received wrong USBTMC header size");

    // Check MsgID field
    if ( t_message_id != t_message[0] ) {
        DEBUG_PRINT("Wrong MsgID returned : expected 0x%02X, received 0x%02X\n",
            t_message_id, t_message[0]);
        throw BadProtocol(this->getInfo() + " - Wrong MsgID received");
    }

    // Check bTag and ~bTag fields
    uint8_t inv_cur_tag = (uint8_t)(~m_cur_tag);
    if ( (t_message[1] != m_cur_tag) || (t_message[2] != inv_cur_tag) ) {
        DEBUG_PRINT("Wrong bTag/~bTag returned : expected 0x%02X/0x%02X, "
            "received 0x%02X/0x%02X\n", m_cur_tag, inv_cur_tag, t_message[1], 
            t_message[2]);
        throw BadProtocol(this->getInfo() + " - Wrong bTag/~bTag received");
    }

    // Check transfer size
    uint32_t transfer_size = ((uint32_t)t_message[4] <<  0) |
                             ((uint32_t)t_message[5] <<  8) |
                             ((uint32_t)t_message[6] << 16) |
                             ((uint32_t)t_message[7] << 24);

    DEBUG_PRINT("MsgID 0x%02X, bTag 0x%02X/0x%02X, TransferSize 0x%08X (%u)\n",
        t_message_id, m_cur_tag, inv_cur_tag, transfer_size, transfer_size);
    return transfer_size;
}

}
