#include <labkit/devices/jenny-science/xenaxxvi75v8.hh>

#include <unistd.h>
#include <sys/time.h>
#include <tuple>
#include <iostream>
#include <sstream>
#include <iomanip>

#include <labkit/exceptions.hh>
#include <labkit/utils.hh>
#include <labkit/debug.hh>

using namespace std;

namespace labkit {

XenaxXvi75V8::XenaxXvi75V8(std::unique_ptr<SerialComm> t_ser) : XenaxXvi75V8()
{
    this->connect(std::move(t_ser));
    return;
}

XenaxXvi75V8::XenaxXvi75V8(std::unique_ptr<TcpipComm> t_tcpip) : XenaxXvi75V8()
{
    this->connect(std::move(t_tcpip));
    return;
}

XenaxXvi75V8::~XenaxXvi75V8()
{
    if (this->connected())
        this->disconnect();
    return;
}

void XenaxXvi75V8::connect(std::unique_ptr<TcpipComm> t_tcpip)
{
    this->setComm(std::move(t_tcpip));
    this->init();
    return;
}

void XenaxXvi75V8::connect(std::unique_ptr<SerialComm> t_ser)
{
    this->setComm(std::move(t_ser));
    this->init();
    return;
}

void XenaxXvi75V8::disconnect()
{
    this->stopMotion();
    return;
}

void XenaxXvi75V8::referenceAxis() 
{
    DEBUG_PRINT("%s\n", "Referencing axis...");
    this->queryCmd("REF");
    return;
}

void XenaxXvi75V8::setReferenceDir(ReferenceDirection t_dir)
{
    DEBUG_PRINT("Changing reference direction to %u\n", t_dir);
    this->queryCmd("DRHR" + to_string(t_dir));
    return;
}

XenaxXvi75V8::ReferenceDirection XenaxXvi75V8::getReferenceDir()
{
    bool success = false;
    unsigned tmp = convertTo<unsigned>(this->queryCmd("DRHR?"));
    ReferenceDirection dir = static_cast<ReferenceDirection>(tmp);
    DEBUG_PRINT("Current reference direction %u\n", dir);
    return dir;
}

int XenaxXvi75V8::getPosition() { 
    bool success = false;
    unsigned ret = convertTo<unsigned>(this->queryCmd("TP"));
    return ret;
}


bool XenaxXvi75V8::forceLimitReached() 
{
    return (this->getStatusRegister() & I_FORCE_LIMIT_REACHED);
}

void XenaxXvi75V8::stopMotion() 
{
    this->queryCmd("SM");
    this->waitStatusClr(IN_MOTION, 200);
    return;
}

bool XenaxXvi75V8::motionCompleted()
{
    uint32_t sreg = 0x0000;
    try{
        sreg = this->getStatusRegister();
    } catch (const DeviceError& ex) {
        switch ( ex.errorNumber() ) {  // Check error codes (manual p. 34)
            case 03:    // In motion
            case 05:    // Program is active
            case 34:    // Rotational reference active
            case 36:    // Linear reference active
            case 38:    // Gantry reference active
            return false;
            break; 
            
            default:
            throw;
        }
    }
    // IN_POSITION set, IN_MOTION cleared
    bool ret = (sreg & IN_POSITION) && !(sreg & IN_MOTION);
    return ret;
}

bool XenaxXvi75V8::referenceCompleted()
{
    uint32_t sreg = 0x0000;
    try{
        sreg = this->getStatusRegister();
    } catch (const DeviceError& ex) {
        switch ( ex.errorNumber() ) {  // Check error codes (manual p. 34)
            case 03:    // In motion
            case 05:    // Program is active
            case 34:    // Rotational reference active
            case 36:    // Linear reference active
            case 38:    // Gantry reference active
                return false;
            default:
                throw;
        }
        throw;
    }
    // REF set, IN_POSITION set, IN_MOTION cleared
    bool ret = (sreg & REF) && (sreg & IN_POSITION) && !(sreg & IN_MOTION);
    return ret;
}

bool XenaxXvi75V8::gantryInitialized() 
{
    return (this->getStatusRegister() & END_OF_GANTRY_INIT);
}

bool XenaxXvi75V8::errorPending()
{
    // Reading the status register also updates m_error_pending
    this->getStatusRegister();
    return m_error_pending;
}

uint32_t XenaxXvi75V8::getStatusRegister() 
{
    string resp = this->queryCmd("TPSR");
    uint32_t status = stoi(resp, 0 , 16);

    DEBUG_PRINT("status register = 0x%08X\n", status);

    // Check error, warning, and info bit => update error pending
    m_error_pending = (status & (ERROR | WARNING | INFO));

    return status;
}

void XenaxXvi75V8::setSpeed(unsigned t_inc_per_sec) 
{
    // Minimum speed is 10 incs per sec!
    unsigned speed = max(10u, t_inc_per_sec);
    this->queryCmd("SP" + to_string(speed));
    return;
}

unsigned XenaxXvi75V8::getSpeed() 
{
    bool success = false;
    unsigned ret = convertTo<unsigned>(this->queryCmd("SP?"));
    return ret; 
}

void XenaxXvi75V8::setAcceleration(unsigned t_inc_per_sec2)
{
    this->queryCmd("AC" + to_string(t_inc_per_sec2));
    return;
}

unsigned XenaxXvi75V8::getAcceleration() 
{ 
    bool success = false;
    unsigned ret = convertTo<unsigned>(this->queryCmd("AC?"));
    return ret; 
}

void XenaxXvi75V8::setSCurve(unsigned t_percent) 
{
    if (t_percent > 100) {
        printf("Invalid S-curve percentage value %i\n", t_percent);
        abort();
    }
    this->queryCmd("SCRV" + to_string(t_percent));
    return;
}

unsigned XenaxXvi75V8::getSCurve() 
{ 
    bool success = false;
    unsigned ret = convertTo<unsigned>(this->queryCmd("SCRV?"));
    return ret;  
}


void XenaxXvi75V8::forceCalibration(unsigned t_len) 
{
    DEBUG_PRINT("%s\n", "Performing force calibration...");
    this->queryCmd("FC" + to_string(t_len));
    this->waitStatusClr(FORCE_CALIBRATION_ACTIVE, 500, 30000);
    this->getForceConstant();
    return;
}

int XenaxXvi75V8::getMotorCurrent() 
{ 
    bool success = false;
    unsigned ret = convertTo<unsigned>(this->queryCmd("TMC"));
    return ret;
}


float XenaxXvi75V8::getForceConstant() 
{
    bool success = false;
    m_force_const = 1e-6 * convertTo<int>(this->queryCmd("FCM?"));
    DEBUG_PRINT("force constant = %f N/mA\n", m_force_const);
    return m_force_const;
}

float XenaxXvi75V8::getMotorForce() 
{
    // TODO: check valid force calibration using "FCV" (manual p. 53)?
    return m_force_const*this->getMotorCurrent();
}

void XenaxXvi75V8::setForceLimit(float fmax_N) 
{
    int flim_10mA = int(0.1*fmax_N/m_force_const);
    this->queryCmd("LIF" + to_string(flim_10mA));
    return;
}

float XenaxXvi75V8::getForceLimit() 
{
    bool success = false;
    int flim_10mA = convertTo<int>(this->queryCmd("LIF?"));
    float flim = 10.*flim_10mA*m_force_const;
    DEBUG_PRINT("force limit = %.3f N\n", flim);
    return flim;
}

void XenaxXvi75V8::setOutputType(unsigned t_output_no, OutputType t_type) 
{
    if ( (t_output_no > 8) || (t_output_no < 1) ){
        fprintf(stderr, "GPIO output number has to be between 1 and 8.\n");
        abort();
    }
    // Clear bits for given gpio number
    m_output_type &= ~(0b11 << 2*(t_output_no-1));
    // Set bits corresponding to state
    m_output_type |= (t_type << 2*(t_output_no-1));
    this->setOutputTypeReg(m_output_type);
    return;
}

void XenaxXvi75V8::setLimits(unsigned t_left, unsigned t_right)
{
    this->queryCmd("LL" + to_string(t_left));
    this->queryCmd("LR" + to_string(t_right));
    return;
}

unsigned XenaxXvi75V8::getLimitLeft() 
{ 
    bool success = false;
    unsigned ret = convertTo<unsigned>(this->queryCmd("LL?"));
    return ret;
}
unsigned XenaxXvi75V8::getLimitRight() 
{ 
    bool success = false;
    unsigned ret = convertTo<unsigned>(this->queryCmd("LR?"));
    return ret;
}

void XenaxXvi75V8::setOutputActivity(unsigned t_output_no, bool t_active_hi) 
{
    if ( (t_output_no > 8) || (t_output_no < 1) ){
        fprintf(stderr, "GPIO output number has to be between 1 and 8.\n");
        abort();
    }
    // Clear bits
    m_output_activity &= ~(0b1 << (t_output_no-1));
    // Set bits
    if (t_active_hi)
        m_output_activity |= (0b1 << (t_output_no-1));
    this->setOutputStateReg(m_output_activity);
    return;
}

void XenaxXvi75V8::setOutput(unsigned t_output_no, bool t_high)
{
    if ( (t_output_no > 8) || (t_output_no < 1) ){
        fprintf(stderr, "GPIO output number has to be between 1 and 8.\n");
        abort();
    }
    if (t_high)
        this->queryCmd("SO" + to_string(t_output_no));
    else 
        this->queryCmd("CO" + to_string(t_output_no));
    return;
}

bool XenaxXvi75V8::getOutput(unsigned t_output_no)
{
    if ( (t_output_no > 8) || (t_output_no < 1) ){
        fprintf(stderr, "GPIO output number has to be between 1 and 8.\n");
        abort();
    }
    uint8_t output_reg = this->getOutputStateReg();
    return ( (1 << (t_output_no - 1)) & output_reg );
}

bool XenaxXvi75V8::getInput(unsigned t_input_no)
{
    if ( (t_input_no > 16) || (t_input_no < 1) ){
        fprintf(stderr, "GPIO input number has to be between 1 and 16.\n");
        abort();
    }
    uint16_t input_reg = this->getInputStateReg();
    return ( (1 << (t_input_no - 1)) & input_reg );
}

void XenaxXvi75V8::setServoId(std::string t_sid)
{
    if (t_sid.size() > 16) {
        fprintf(stderr, "SID '%s' exceeds max. length of 16 characters.\n",
            t_sid.c_str());
        abort();
    }
    this->queryCmd("SID" + t_sid);
    return;
}

void XenaxXvi75V8::setCardId(unsigned t_cid)
{
    if (t_cid > 4) {
        fprintf(stderr, "Invalid card identifier %u (valid 0 - 4)\n", t_cid);
        abort();
    }
    DEBUG_PRINT("Changing card identifier to %i\n", t_cid);
    this->queryCmd("CI" + to_string(t_cid));
    return;
}

unsigned XenaxXvi75V8::getCardId()
{
    bool success = false;
    unsigned cid = convertTo<unsigned>(this->queryCmd("CI?"));
    DEBUG_PRINT("Read card identifier %i\n", cid);
    return cid;
}

void XenaxXvi75V8::setGantrySlaveId(unsigned t_gsid)
{
    DEBUG_PRINT("Changing grantry slave identifier to %i\n", t_gsid);
    this->queryCmd("GSID" + to_string(t_gsid));
    return;
}

unsigned XenaxXvi75V8::getGantrySlaveId()
{
    bool success = false;
    unsigned gsid = convertTo<unsigned>(this->queryCmd("GSID?"));
    DEBUG_PRINT("Read card identifier %i\n", gsid);
    return gsid;
}

void XenaxXvi75V8::setGantryMasterSlaveOffs(int t_gmso)
{
    DEBUG_PRINT("Setting gantry master slave offset to %i\n", t_gmso);
    this->queryCmd("PGMSO" + to_string(t_gmso));
    return;
}

int XenaxXvi75V8::getGantryMasterSlaveOffs()
{
    bool success = false;
    unsigned gmso = convertTo<unsigned>(this->queryCmd("PGMSO?"));
    DEBUG_PRINT("Read gantry master slave offset %i\n", gmso);
    return gmso;
}

int XenaxXvi75V8::detectedGantryMasterSlaveOffs()
{
    bool success = false;
    unsigned dgmso = convertTo<unsigned>(this->queryCmd("DGMSO?"));
    DEBUG_PRINT("Read detected gantry master slave offset %i\n", dgmso);
    return dgmso;
}

unsigned XenaxXvi75V8::getError(std::string &t_strerror) 
{
    // Toggle error pending
    if (m_error_pending)
        m_error_pending = false;
    bool success = false;
    m_error = convertTo<int>(this->queryCmd("TE"));
    t_strerror = this->queryCmd("TES");
    return m_error;
}

void XenaxXvi75V8::setPayload(unsigned t_payload_g)
{
    DEBUG_PRINT("Setting payload to %u\n", t_payload_g);
    this->queryCmd("ML" + to_string(t_payload_g));
    return;
}

unsigned XenaxXvi75V8::getPayload()
{
    bool success = false;
    unsigned payload = convertTo<int>(this->queryCmd("ML?"));
    DEBUG_PRINT("Read payload %u\n", payload);
    return payload;
}

void XenaxXvi75V8::setGainPos(unsigned t_gain_pos)
{
    DEBUG_PRINT("Setting GAIN POS to %u\n", t_gain_pos);
    this->queryCmd("BWP" + to_string(t_gain_pos));
    return;
}

unsigned XenaxXvi75V8::getGainPos()
{
    bool success = false;
    unsigned gain_pos = convertTo<int>(this->queryCmd("BWP?"));
    DEBUG_PRINT("Read GAIN POS %u\n", gain_pos);
    return gain_pos;
}

void XenaxXvi75V8::setGainCur(unsigned t_gain_cur)
{
    DEBUG_PRINT("Setting GAIN CUR to %u\n", t_gain_cur);
    this->queryCmd("BWC" + to_string(t_gain_cur));
}

unsigned XenaxXvi75V8::getGainCur()
{
    bool success = false;
    unsigned gain_cur = convertTo<int>(this->queryCmd("BWC?"));
    DEBUG_PRINT("Read GAIN CUR %u\n", gain_cur);
    return gain_cur;
}

void XenaxXvi75V8::setMaxDeviation(unsigned t_max_dev)
{
    DEBUG_PRINT("Setting Deviation POS ACT to %u\n", t_max_dev);
    this->queryCmd("DP" + to_string(t_max_dev));
    return;
}

unsigned XenaxXvi75V8::getMaxDeviation()
{
    bool success = false;
    unsigned max_dev = convertTo<int>(this->queryCmd("DP?"));
    DEBUG_PRINT("Read Deviation POS ACT %u\n", max_dev);
    return max_dev;
}

void XenaxXvi75V8::setTargetDeviation(unsigned t_tar_dev)
{
    DEBUG_PRINT("Setting Deviation TARGET to %u\n", t_tar_dev);
    this->queryCmd("DTP" + to_string(t_tar_dev));
    return;
}

unsigned XenaxXvi75V8::getTargetDeviation()
{
    bool success = false;
    unsigned tar_dev = convertTo<int>(this->queryCmd("DTP?"));
    DEBUG_PRINT("Read Deviation TARGET %u\n", tar_dev);
    return tar_dev;
}

tuple<unsigned,string> XenaxXvi75V8::getError()
{
    string error_str;
    unsigned error_no = this->getError(error_str);
    return make_tuple<unsigned,string>(std::move(error_no), std::move(error_str));
}

/*
 *      P R I V A T E   M E T H O D S
 */

void XenaxXvi75V8::init() 
{
    // Clear input buffer
    this->flushBuffer();
    // Disable asynchronous status updates
    this->queryCmd("EVT0");
    // Get force constant for force calculations
    this->getForceConstant();
    // Get servo identifier
    string servo_id = this->queryCmd("SID?");
    if (!servo_id.empty())
        m_name += "," + servo_id;
    return;
}

void XenaxXvi75V8::flushBuffer() 
{
    DEBUG_PRINT("%s\n", "flushing read buffer...");
    while (true) {
        try { this->getComm()->read(200); }
        catch (const Timeout &ex) { break; }
    }
    m_input_buffer.clear();
    DEBUG_PRINT("%s\n", "buffer flushed");
    return;
}

string XenaxXvi75V8::queryCmd(string t_cmd, unsigned t_timeout_ms) 
{
    // Send command and CR
    this->getComm()->write(t_cmd + "\n");
    size_t pos;

    // Read until an EOM delimiter was received and store in buffer
    // (see application note 'TCP_IP_KOMMUNIKATION.pdf' p. 1)
    m_input_buffer.append( this->getComm()->readUntil(">", pos, t_timeout_ms) );

    // Split response into parameters and remove it from the buffer
    string resp = m_input_buffer.substr(0, pos);
    vector<string> par_list = split(resp, "\r\n", 10);
    m_input_buffer.erase(0, pos+1);

    #ifdef LD_DEBUG
    for (size_t i = 0; i < par_list.size(); i++) {
        DEBUG_PRINT("param (%zu/%zu): '%s'\n", i+1, par_list.size(),
            par_list.at(i).c_str());
    }
    #endif

    // First parameter should be cmd echo
    if ( par_list.at(0).find(t_cmd) == string::npos )
        throw BadProtocol("No command echo for '" + t_cmd + "' received");
    // Second parameter is the payload
    if (par_list.size() == 2) {
        if ( par_list.at(1).find("?") != string::npos)
            throw BadProtocol("Invalid command '" + t_cmd + "'\n");
        if ( par_list.at(1).find("#") != string::npos) {
            // Handle hash error message (refer manual p.34)
            string hash_str = par_list.at(1).substr(1);
            int hash_no = stoi(hash_str);
            switch (hash_no) {
                case 01:
                throw DeviceError("Cannot execute command, error in error "
                "queue (" + par_list.at(1) + ")\n", hash_no);
                break;

                case 03:
                throw DeviceError("Cannot execute command, currently moving"
                " (" + par_list.at(1) + ")\n", hash_no);
                break;

                case 05:
                throw DeviceError("Cannot execute command, program is active"
                " (" + par_list.at(1) + ")\n", hash_no);
                break;

                case 13:
                throw DeviceError("Cannot execute command, emergency exit "
                "EE1 is pending (" + par_list.at(1) + ")\n", hash_no);

                case 14:
                throw DeviceError("Cannot execute command, emergency exit "
                "EE is pending (" + par_list.at(1) + ")\n", hash_no);

                case 15:
                throw DeviceError("Cannot execute command, force calibration "
                "is active (" + par_list.at(1) + ")\n", hash_no);
                break;

                case 27:
                throw DeviceError("Cannot execute command, I Force Drift "
                "Compensation is active (" + par_list.at(1) + ")\n", hash_no);
                break;

                case 34:
                throw DeviceError("Cannot execute command, rotation reference "
                "is active (" + par_list.at(1) + ")\n", hash_no);
                break;

                case 36:
                throw DeviceError("Cannot execute command, gantry reference "
                "is active (" + par_list.at(1) + ")\n", hash_no);
                break;

                case 38:
                throw DeviceError("Cannot execute command, reference is "
                "active (" + par_list.at(1) + ")\n", hash_no);
                break;

                case 40:
                throw DeviceError("Cannot execute command, command not "
                "permitted (" + par_list.at(1) + ")\n", hash_no);
                break;

                case 47:
                throw DeviceError("Cannot execute command, fault reaction "
                "active (" + par_list.at(1) + ")\n", hash_no);
                break;

                case 49:
                throw DeviceError("Cannot execute command, no JSC motor "
                "connected (" + par_list.at(1) + ")\n", hash_no);
                break;

                case 65:
                throw DeviceError("Cannot execute command, parameter out of "
                "value range (" + par_list.at(1) + ")\n", hash_no);
                break;

                case 66:
                throw DeviceError("Cannot execute command, 5s timeout occured "
                "(" + par_list.at(1) + ")\n", hash_no);
                break;

                default:
                throw DeviceError("Cannot execute command, unknown hash (" 
                + par_list.at(1) + ")\n", hash_no);
                break;
            }
        }

        return par_list.at(1);
    }

    return "";
}

void XenaxXvi75V8::waitStatusSet(uint32_t t_status, unsigned t_interval_ms,
unsigned t_timeout_ms) 
{
    // Setup timeout
    struct timeval tsta, tsto;
    gettimeofday(&tsta, NULL);
    float tdiff;
    uint32_t sreg = this->getStatusRegister();

    // Wait until all masked bits in status are set to '1'
    while ( (sreg & t_status) != t_status ) {
        usleep(1e3 * t_interval_ms);

        // Check for timeout
        gettimeofday(&tsto, NULL);
        tdiff = (tsto.tv_sec - tsta.tv_sec) * 1000.;
        if (tdiff > t_timeout_ms) {
            stringstream msg;
            msg << "Timeout on SREG bits to set" << endl;
            msg << "Expected 0x" << uppercase << setfill('0') << setw(8) 
                << hex << t_status << "to set" << endl; 
            msg << "Current  0x" << uppercase << setfill('0') << setw(8) 
                << hex << sreg << endl; 
            throw Timeout(msg.str());
        }
        sreg = this->getStatusRegister();
    }

    return;
}

void XenaxXvi75V8::waitStatusClr(uint32_t t_status, unsigned t_interval_ms,
    unsigned t_timeout_ms) 
{
    // Setup timeout
    struct timeval tsta, tsto;
    gettimeofday(&tsta, NULL);
    float tdiff;
    uint32_t sreg = this->getStatusRegister();

    // Wait until all masked bits in status are cleared to '0'
    while ( ( sreg & t_status) ) {
        usleep(t_interval_ms*1000);

        // Check for timeout
        gettimeofday(&tsto, NULL);
        tdiff = (tsto.tv_sec - tsta.tv_sec) * 1000.;
        if (tdiff > t_timeout_ms) {
            stringstream msg;
            msg << "Timeout on SREG bits to clear" << endl;
            msg << "Expected 0x" << uppercase << setfill('0') << setw(8) 
                << hex << t_status  << " to clear" << endl; 
            msg << "Current  0x" << uppercase << setfill('0') << setw(8) 
                << hex << sreg << endl; 
            throw Timeout(msg.str());
        }
        sreg = this->getStatusRegister();
    }

    return;
}

void XenaxXvi75V8::setOutputTypeReg(uint16_t t_mask) 
{
    DEBUG_PRINT("Setting output type to 0x%04X\n", t_mask);
    this->queryCmd("SOT" + to_string(t_mask));
    return;
}

void XenaxXvi75V8::setOutputStateReg(uint8_t t_mask) 
{
    DEBUG_PRINT("Setting output state to 0x%02X\n", t_mask);
    this->queryCmd("SOA" + to_string(t_mask));
    return;
}

}
