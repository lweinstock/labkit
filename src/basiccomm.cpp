#include <labkit/comms/basiccomm.hh>
#include <labkit/exceptions.hh>
#include <labkit/debug.hh>

#include <sys/time.h>

using namespace std;

namespace labkit
{

void BasicComm::writeByte(const vector<uint8_t> data)
{
    this->writeRaw(data.data(), data.size());
    return;
}

void BasicComm::write(const string& msg) 
{
    const uint8_t* wbuf = reinterpret_cast<const uint8_t*>(&msg[0]);
    this->writeRaw(wbuf, msg.size());
    DEBUG_PRINT_STRING_DATA(msg, "Sent %zu bytes: ", msg.size());
    return;
}

vector<uint8_t> BasicComm::readByte(size_t max_len, unsigned timeout_ms)
{
    vector<uint8_t> rbuf(DFLT_BUF_SIZE);
    max_len = min(max_len, DFLT_BUF_SIZE);  // Limited size
    ssize_t nbytes = this->readRaw(&rbuf[0], max_len, timeout_ms);
    return vector<uint8_t>(rbuf.begin(), rbuf.begin() + nbytes);
}

string BasicComm::read(unsigned timeout_ms) 
{
    vector<uint8_t> rbuf(DFLT_BUF_SIZE);
    ssize_t nbytes = this->readRaw(&rbuf[0], DFLT_BUF_SIZE, timeout_ms);
    string ret(rbuf.begin(), rbuf.begin() + nbytes);
    DEBUG_PRINT_STRING_DATA(ret, "Read %zu bytes: ", ret.size());
    return ret;
}

string BasicComm::readUntil(const string& delim, size_t& pos, 
    unsigned timeout_ms) 
{
    string ret("");
    struct timeval sta, sto;
    gettimeofday(&sta, NULL);
    do {
        string rbuf = this->read(timeout_ms);
        if (rbuf.size() > 0)
            ret.append(rbuf);
        pos = ret.rfind(delim);

        // Check timeout
        gettimeofday(&sto, NULL);
        unsigned diff_ms = (sto.tv_sec-sta.tv_sec)*1000 
            + (sto.tv_usec-sta.tv_usec)/1000;
        if (diff_ms > timeout_ms) 
            throw Timeout("Did not receive delimiter '" + delim + "' in time");
    } while ( pos == string::npos );
    return ret;
}

string BasicComm::readUntil(const string& delim, unsigned timeout_ms) 
{
    size_t temp {0};
    return this->readUntil(delim, temp, timeout_ms);    
}

string BasicComm::query(const string& msg, unsigned timeout_ms, 
    unsigned t_delay_ms) 
{
    this->write(msg);
    if (t_delay_ms) usleep(1e3 * t_delay_ms);
    return this->read(timeout_ms);
}

string BasicComm::queryUntil(const string& t_msg, const string& t_delim, 
    unsigned t_timeout_ms, unsigned t_delay_ms)
{
    this->write(t_msg);
    if (t_delay_ms) usleep(1e3 * t_delay_ms);
    return this->readUntil(t_delim, t_timeout_ms);
}

vector<uint8_t> BasicComm::queryByte(const vector<uint8_t> data, 
    unsigned timeout_ms, unsigned t_delay_ms)
{
    this->writeByte(data);
    if (t_delay_ms) usleep(1e3 * t_delay_ms);
    return this->readByte(timeout_ms);
}

}