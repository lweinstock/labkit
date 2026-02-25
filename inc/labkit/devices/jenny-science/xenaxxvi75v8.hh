#ifndef LK_XENAX_XVI_HH
#define LK_XENAX_XVI_HH

#include <labkit/devices/basicdevice.hh>
#include <labkit/comms/tcpipcomm.hh>
#include <labkit/comms/serialcomm.hh>

#include <memory>

namespace labkit {

class XenaxXvi75V8 : public BasicDevice {
public:
    XenaxXvi75V8() : BasicDevice("XENAX Xvi 75v8"), m_force_const(0), 
        m_error(0), m_output_type(0x5555), m_output_activity(0xFF), 
        m_error_pending(false) {};
    XenaxXvi75V8(std::unique_ptr<TcpipComm> t_tcpip);
    XenaxXvi75V8(std::unique_ptr<SerialComm> t_ser);
    ~XenaxXvi75V8();

    // XENAX default port 10001
    static constexpr unsigned PORT = 10001;

    /// Connect to Xenax Xvi 75v8 via TCP/IP
    void connect(std::unique_ptr<TcpipComm> t_tcpip);

    /// Connect to Xenax Xvi 75v8 via USBTMC
    void connect(std::unique_ptr<SerialComm> t_ser);

    /// Stop motion and disconnect from Xenax Xvi 75v8
    void disconnect() override;

    // En-/disable power of Xenax motor controller
    void powerOn(bool t_enable = true) { this->queryCmd( t_enable? "PW" : "PQ"); }
    void powerOff() { powerOn(false); }
    void powerContinue() { this->queryCmd("PWC"); }
    bool isOn() { return std::stoi(this->queryCmd("TS")); }

    // Referencing for absolute position measurements (see manual p.48)
    void referenceAxis();
    // Change reference direction (see manual p. 48)
    enum ReferenceDirection : int {REF_POS = 0, REF_NEG, GANTRY_POS, 
        GANTRY_NEG, GANTRY_POS_NEG, GANTRY_NEG_POS};
    void setReferenceDir(ReferenceDirection dir);
    ReferenceDirection getReferenceDir();

    // Go to absolute position in micro meter
    void movePosition(int t_pos) { this->queryCmd("G" + std::to_string(t_pos)); }
    int getPosition();
    // Move in positive/negative direction with constant speed
    void jogPos() { this->queryCmd("JP"); }
    void jogNeg() { this->queryCmd("JN"); }
    void stopMotion();
    // Status queries
    bool motionCompleted();
    bool referenceCompleted();
    bool gantryInitialized();
    bool errorPending();
    
    // Read the Process Status Register (PSR) & update status
    uint32_t getStatusRegister();
    // Process Status Register definition (manual p. 56)
    enum ProcessStatusRegister : uint32_t {
        ERROR                     = (1 << 0),
        REF                       = (1 << 1),
        IN_MOTION                 = (1 << 2),
        IN_POSITION               = (1 << 3),
        END_OF_PROGRAM            = (1 << 4),
        IN_FORCE                  = (1 << 5),
        IN_SECTO                  = (1 << 6),
        FORCE_IN_SECTOR           = (1 << 7),
        INVERTER_VOLTAGE          = (1 << 8),
        END_OF_GANTRY_INIT        = (1 << 9),
        NEGATIVE_LIMIT_SWITC      = (1 << 10),
        POSITIVE_LIMIT_SWITC      = (1 << 11),
        REMAIN_POWER_ON           = (1 << 12),
        POWER_OFF                 = (1 << 13),
        FORCE_CALIBRATION_ACTIVE  = (1 << 14),
        I_FORCE_LIMIT_REACHED     = (1 << 15),
        STO_PRIMED_HIT            = (1 << 16),
        SS1_PRIMED_HIT            = (1 << 17),
        SS2_PRIMED                = (1 << 18),
        SS2_HIT                   = (1 << 19),
        SLS_PRIMED                = (1 << 20),
        SLS_SPEED_HIT             = (1 << 21),
        SLS_POSITION_HIT          = (1 << 22),
        WARNING                   = (1 << 23),
        INFO                      = (1 << 24),
        PHASING_DONE              = (1 << 25),
        I_FORCE_DRIFT_COMP_ACTIVE = (1 << 26)
    };

    // Set and get movement parameters;
    //   speed [inc/s], accel [inc/s2], s curve [%]
    void setSpeed(unsigned t_inc_per_sec);
    unsigned getSpeed();
    void setAcceleration(unsigned t_inc_per_sec2);
    unsigned getAcceleration();
    void setSCurve(unsigned t_percent);
    unsigned getSCurve();

    // Calibration for more precise force measurements (dF ~ 0.5 - 1.0 N)
    void forceCalibration(unsigned t_distance);
    // Motor current and force information;
    //  current [mA], force const [N/mA], force [N]
    int getMotorCurrent();
    float getForceConstant();
    float getMotorForce();
    // Set force limit (manual p. 54)
    void setForceLimit(float t_fmax_n);
    float getForceLimit();
    bool forceLimitReached();

    // Set and get soft limits (left = min, right = max)
    void setLimits(unsigned t_left, unsigned t_right);
    unsigned getLimitLeft();
    unsigned getLimitRight();

    enum OutputType : uint8_t {
        SINK = 0b00,
        SOURCE = 0b01,
        SINK_SOURCE = 0b10
    };

    // Programmable Logic Controller (PLC) GPIO settings (manual p. 51ff)
    void setOutputType(unsigned t_output_no, OutputType t_type);
    void setOutputActivity(unsigned t_output_no, bool t_active_hi);
    void setOutput(unsigned t_output_no, bool t_high);
    bool getOutput(unsigned t_output_no);
    // Returns true when high, false when low
    bool getInput(unsigned t_input_no);
    // Returns raw hex input and output registers
    uint8_t getOutputStateReg() 
        { return static_cast<uint8_t>(std::stoi(this->queryCmd("TOX"), 0, 16)); }
    uint16_t getInputStateReg() 
        { return static_cast<uint8_t>(std::stoi(this->queryCmd("TIX"), 0, 16)); }

    // Motor type reset (in response to error 59)
    void resetMotorType() { this->queryCmd("RESM", 10000); }
    // Disable motion blocked by unconfigured Safety Motion Unit (SMU)
    void disableSMU() { this->queryCmd("DMBUS"); }

    // Set custom servo identifier (max. 16 bytes)
    void setServoId(std::string t_sid);
    void resetServoId() { this->setServoId(""); }
    std::string getServoId() { return this->queryCmd("SID?"); }

    // Set card identifier (used for master/slave + gantry, manual p. 94)
    void setCardId(unsigned t_cid);
    unsigned getCardId();
    // Set gantry slave id in master controller
    void setGantrySlaveId(unsigned t_gsid);
    unsigned getGantrySlaveId();
    // Set/get gantry offset master/slave
    void setGantryMasterSlaveOffs(int t_gmso);
    int getGantryMasterSlaveOffs();
    int detectedGantryMasterSlaveOffs();

    // Controller settings (manual p. 46/47)
    void setPayload(unsigned t_payload_g);
    unsigned getPayload();
    void setGainPos(unsigned t_gain_pos);
    unsigned getGainPos();
    void setGainCur(unsigned t_gain_cur);
    unsigned getGainCur();
    void setMaxDeviation(unsigned t_max_dev);
    unsigned getMaxDeviation();
    void setTargetDeviation(unsigned tt_ar_dev);
    unsigned getTargetDeviation();

    // Get error information (C- and python-style)
    unsigned getError(std::string &t_strerror);
    std::tuple<unsigned,std::string> getError();

private:
    std::string m_input_buffer;
    float m_force_const;   // I->F conversion factor [N/mA]
    int m_error;
    uint16_t m_output_type;
    uint8_t m_output_activity;
    bool m_error_pending;

    // GPIO Set Output Type (SOT) definitions (manual p. 26)
    enum SOT : uint16_t {
        SOT10 = (1 << 0),
        SOT11 = (1 << 1),
        SOT20 = (1 << 2),
        SOT21 = (1 << 3),
        SOT30 = (1 << 4),
        SOT31 = (1 << 5),
        SOT40 = (1 << 6),
        SOT41 = (1 << 7),
        SOT50 = (1 << 8),
        SOT51 = (1 << 9),
        SOT60 = (1 << 10),
        SOT61 = (1 << 11),
        SOT70 = (1 << 12),
        SOT71 = (1 << 13),
        SOT80 = (1 << 14),
        SOT81 = (1 << 15)
    };

    // GPIO Set Output Activity (SOA) definitions (manual p. 26)
    enum SOA : uint8_t {
        SOA1 = (1 << 0),
        SOA2 = (1 << 1),
        SOA3 = (1 << 2),
        SOA4 = (1 << 3),
        SOA5 = (1 << 4),
        SOA6 = (1 << 5),
        SOA7 = (1 << 6),
        SOA8 = (1 << 7)
    };

    void init();
    void flushBuffer();

    // General command query
    std::string queryCmd(std::string t_cmd, unsigned t_timeout_ms = 1000);

    // Wait until status bits are set
    void waitStatusSet(uint32_t t_status, unsigned t_interval_ms = 500,
        unsigned t_timeout_ms = 10000);

    // Wait until status bits are cleared
    void waitStatusClr(uint32_t t_status, unsigned t_interval_ms = 500,
        unsigned t_timeout_ms = 10000);

    // GPIO register access
    void setOutputTypeReg(uint16_t t_mask);
    void setOutputStateReg(uint8_t t_mask);

    void readErrorQueue();

};

}

#endif
