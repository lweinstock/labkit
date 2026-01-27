#ifndef LK_BASIC_COMM_HH
#define LK_BASIC_COMM_HH

#include <cstdint>
#include <string>
#include <vector>

namespace labkit
{

/// Enum for the communication interface types
enum CommType {NONE, SERIAL, TCPIP, USB, USBTMC};

/** \brief Abstract base class for all communication interfaces.
 *
 *  This class describes basic functionalities that have to be implemented by
 *  all communication interfaces; a ctor to establish communication, basic read
 *  and write functions, and a dtor to close communication. 
 *  
 *  Copy ctor and assignment operator are removed since a communication
 *  interface represents a physical connection to a device or instrument and
 *  is therefore unique.
 */
class BasicComm {
public:
    /// Default constructor
    BasicComm() : m_good(false) {};
    /// Default destructor
    virtual ~BasicComm() {};

    /// No copy constructor; comm interfaces are unique physical entities
    BasicComm(const BasicComm&) = delete;
    /// No assignment operator; comm interfaces are unique physical entities
    BasicComm& operator=(const BasicComm&) = delete;

    /// 1MB default buffer size
    static constexpr size_t DFLT_BUF_SIZE = 1024*1024;  
    /// 2s default timeout
    static constexpr unsigned DFLT_TIMEOUT_MS = 2000;

    /** \brief C-style raw byte write.
     *  \param [in] t_data Output byte array.
     *  \param [in] t_len Length of byte array.
     *  \return Number of successfully written bytes.
     */
    virtual int writeRaw(const uint8_t* t_data, size_t t_len) = 0;

    /** \brief C++-style byte write.
     *  \param [in] t_data Output byte vector.
     */
    void writeByte(const std::vector<uint8_t> t_data);

    /** \brief C++-style string write.
     *  \param [in] t_msg Output string.
     */
    void write(const std::string& t_msg);

    /** \brief C-style raw byte read.
     *  \param [out] t_data Input byte array.
     *  \param [in] t_max_len Maximum length of byte array.
     *  \return Number of successfully read bytes.
     */
    virtual int readRaw(uint8_t* t_data, size_t t_max_len, 
        unsigned timeout_ms = DFLT_TIMEOUT_MS) = 0;

    /** \brief C++-style byte read.
     *  \param [in] t_max_len Maximum length of bytes to read.
     *  \param [in] t_timeout_ms Read timeout in milli seconds.
     *  \return Vector with filled bytes.
     */
    std::vector<uint8_t> readByte(size_t t_max_len = DFLT_BUF_SIZE, 
        unsigned t_timeout_ms = DFLT_TIMEOUT_MS);

    /** \brief C++-style string read.
     *  \param [in] t_timeout_ms Read timeout in milli seconds.
     *  \return String composed of read bytes.
     */
    std::string read(unsigned t_timeout_ms = DFLT_TIMEOUT_MS);

    /** \brief Read until specified delimiter is found in the received message.
     *  \param [in] t_delim Stop delimiter.
     *  \param [out] t_pos Position of the delimiter in string.
     *  \param [in] t_timeout_ms Read timeout in milli seconds.
     *  \return String composed of read bytes.
     */
    std::string readUntil(const std::string& t_delim, size_t& t_pos, 
        unsigned t_timeout_ms = DFLT_TIMEOUT_MS);

    /** \brief Read until specified delimiter is found in the received message.
     *  \param [in] t_delim Stop delimiter.
     *  \param [in] t_timeout_ms Read timeout in milli seconds.
     *  \return String composed of read bytes.
     */
    std::string readUntil(const std::string& t_delim, 
        unsigned timeout_ms = DFLT_TIMEOUT_MS);

    /** \brief C++-style string write followed by a read.
     *  \param [in] t_msg Query message.
     *  \param [in] t_timeout_ms Read timeout in milli seconds.
     *  \return Response string.
     */
    std::string query(const std::string& t_msg, 
        unsigned t_timeout_ms = DFLT_TIMEOUT_MS);

    /**
     * \brief C++-style string write followed by a read until delim is read.
     * \param [in] t_msg Query message.
     * \param [in] t_delim Stop delimiter.
     * \param [in] t_timeout_ms Read timeout in milli seconds.
     * \return Response string
     */
    std::string queryUntil(const std::string& t_msg, const std::string& t_delim, 
        unsigned t_timeout_ms = DFLT_TIMEOUT_MS);

    /** \brief C++-style byte write followed by a read.
     *  \param [in] t_data Query bytes.
     *  \param [in] t_timeout_ms Read timeout in milli seconds.
     *  \return Response bytes.
     */
    std::vector<uint8_t> queryByte(const std::vector<uint8_t> t_data, 
        unsigned t_timeout_ms = DFLT_TIMEOUT_MS);

    /// Open interface with stored settings
    virtual void open() = 0;

    /// Close interface
    virtual void close() = 0;

    /// Returns true if interface is usable.
    virtual bool good() const { return m_good; }

    /// Returns human readable string with information.
    virtual std::string getInfo() const noexcept = 0;

    /// Returns interface type; can be used to break abstraction.
    virtual CommType type() const noexcept = 0;

protected:
    /// Can be set by derived classes if the interface is valid and usable
    bool m_good;
};

    
}

#endif