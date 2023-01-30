#ifndef RMDx8ArduinoUBUV360_h
#define RMDx8ArduinoUBUV360_h

#include "Arduino.h"
#include <mcp_can.h>

class RMDx8ArduinoUBUV360{
public:
    bool wReadWriteFlag;
    unsigned char len;
    unsigned char tmp_buf[8], cmd_buf[8], reply_buf[8], pos_buf[8];
    unsigned char Type1, Type2, Type3, Type4, Type5, Type6, Type7; //ASCII
    int8_t temperature;
    uint8_t CurrKP, CurrKI, SpdKP, SpdKI, PosKP, PosKI, index, command, RlyCtrlRslt, runmode, baudrate, ID;
    int16_t iq, speed, degree, iA, iB, iC, iqControl;
    uint16_t voltage, errorState, maxSpeed, motorpower, CANID, MOTOR_ADDRESS, CAN_MOTOR_ADDRESS;
    uint16_t p_des, v_des, kp, kd, t_ff;
    int32_t encoder, encoderRaw, encoderOffset, motorAngle, speedControl, angleControl;
    uint32_t Accel, SysRunTime, VersionDate, CanRecvTime_MS; 

    RMDx8ArduinoUBUV360(MCP_CAN &CAN, const uint16_t motor_addr); // Maneja el controlador si tiene el mismo nombre que la clase

    // Commands
    void canSetup();
    void readPID(); // 0x30 // Read PID parameter command
    void writePIDRAM(uint8_t CurrKP, uint8_t CurrKI, uint8_t SpdKP, uint8_t SpdKI, uint8_t PosKP, uint8_t PosKI); // 0x31 // Write PID parameters to RAM command 
    void writePIDROM(uint8_t CurrKP, uint8_t CurrKI, uint8_t SpdKP, uint8_t SpdKI, uint8_t PosKP, uint8_t PosKI); // 0x32 // Write PID parameters to ROM command 
    void readAccel(); // 0x42 // Read acceleration data command 
    void writeAccel(uint8_t index, uint32_t Accel); // 0x43 // Write acceleration to RAM and ROM command
    void readMultiTurnEncoderPos(); // 0x60 // Read multi-turn encoder position data command
    void readMultiTurnOriginalPos(); // 0x61 // Read multi-turn encoder original position data command
    void readMultiTurnEncoderZeroOffset(); // 0x62 // Read multi-turn encoder zero offset data command
    void writeEncoderMultiTurnValueROMZero(int32_t encoderOffset); // 0x63 // Write encoder multi-turn value to ROM as motor zero command
    void writeCurrentMultiTurnPositionROMZero(); // 0x64 // Write the current multi-turn position of the encoder to the ROM as the motor zero command
    void readMultiTurnAngle(); // 0x92 // Read multi-turn angle command
    void readMotorStatus1ErrorFlag(); // 0x9A // Read Motor Status 1 and Error Flag Command
    void readMotorStatus2(); // 0x9C // Read Motor Status 2
    void readMotorStatus3(); // 0x9D // Read Motor Status 3
    void motorShutdown(); // 0x80 // Motor shutdown command
    void stopMotor(); // 0x81 // Motor stop command
    void torqueControl(int16_t iqControl); // 0xA1 // Torque closed-loop control command
    void speedClosedLoopControl(int32_t speedControl); // 0xA2 // Speed Closed-loop Control Command
    void positionTrackingControl(int32_t angleControl); // 0xA3 // Position tracking control command
    void absolutePositionControl(uint16_t maxSpeed, int32_t angleControl); // 0xA4 // Absolute position closed-loop control command
    void positionControlSpeedLimit(uint16_t maxSpeed , int32_t angleControl); // 0xA5 // Position tracking control command with speed limit 
    void incrementalPositionControl(uint16_t maxSpeed, int32_t angleControl); // 0xA8 // Incremental position closed-loop control command
    void systemOperatingModeAcquisition(); // 0x70 // Incremental position closed-loop control command
    void motorPowerAcquisition(); // 0x71 // Motor power acquisition command
    void systemReset(); // 0x76 // System reset command
    void systemBrakeRelease(); // 0x77 // System brake release command
    void systemBrakeLock(); // 0x78 // System brake lock command
    void systemRuntimeRead(); // 0xB1 // System runtime read command
    void systemSoftwareVersionDateRead(); // 0xB2 // System software version date read command
    void communicationInterruptionProtectionTimeSetting(uint32_t CanRecvTime_MS); // 0xB3 // Communication interruption protection time setting command
    void communicationbaudRateSetting(uint8_t baudrate); // 0xB4 // Communication baud rate setting command
    void motorModelReading(); // 0xB5 // Motor model reading command
    void functionControl(uint8_t index, uint32_t Value); // 0x20 // Function control command
    void multiMotor(uint8_t command); // 0x280 // Multi-motor command
    void CANIDSetting(bool wReadWriteFlag, uint16_t CANID); // 0x79 // CANID setting command
    void motionModeControlCAN(uint16_t p_des, uint16_t v_des, uint16_t kp, uint16_t kd, uint16_t t_ff); // 0x400 // Motion Mode Control Command_CAN

    // General function
    void serialWriteTerminator();

private:
    MCP_CAN _CAN;
    uint32_t pos_u32t;

    void readBuf(unsigned char *buf);
    void writeCmd(unsigned char *buf);
};

#endif
