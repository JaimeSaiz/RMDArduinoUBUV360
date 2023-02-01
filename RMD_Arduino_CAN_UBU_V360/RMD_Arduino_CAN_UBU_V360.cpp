// SI SE CAMBIA DE TIPO DE DATOS, HAY QUE MIRAR ESA RESTA DERIVADA DEL TIPO DE DATO 
// PARA QUE SEA CORRECTA (VER https://clickhouse.tech/docs/es/sql-reference/data-types/int-uint/).
// SU USO DEPENDE DEL ENCODER (DE 12bits A 18bits)
// -----------------------------
// -----------------------------
// HAY QUE REVISAR LOS TIPOS DE DATOS  int16_t,  uint16_t, uint32_t, int32_t,....
// -----------------------------
// HAY QUE REVISAR LAS ???? PORQUE NO ESTOY SEGURO DE ALGUNOS DATOS.
// HAY QUE REVISAR multiMotor() PORQUE DEBERÍA RECIBIR HASTA 4 RESPESUTAS DE 4 MOTORES DIFERENTES.
// TAMPOCO ESTÁ CLARO CÓMO LLEGAR AL RESTO DE MOTORES
// ----------------------------------------------
// Habrá que ver qué contiene "*buf" dentro de "writeCmd(unsigned char *buf)" porque me falta en la función 0x280.
// -----------------------------


#include "Arduino.h"
#include "RMDArduinoUBUV360.h"
#include <mcp_can.h>

// constructor
RMDArduinoUBUV360::RMDArduinoUBUV360(MCP_CAN &CAN, const uint16_t motor_addr) 
    :_CAN(CAN){
        MOTOR_ADDRESS = motor_addr;
    }

void RMDArduinoUBUV360::canSetup() {
    while (CAN_OK != _CAN.begin(CAN_1000KBPS)) {
        Serial.println("CAN BUS Shield init fail");
        Serial.println("Init CAN BUS Shield again");
        delay(100);
    }
    Serial.println("CAN BUS Shield init ok!");
}

/** 
 * Single motor Command description
 * Read PID parameter command (one frame)
 * This command can read the parameters of current, speed, position loop KP and KI at one time, 
 * and the data type is uint8_t. The system sets the maximum range of PI parameters according 
 * to the motor model, and then divides it equally according to the maximum range of uint8_t 
 * of 256 units. Users only need to adjust 0-256 units. 
*/
void RMDArduinoUBUV360::readPID() {
    cmd_buf[0] = 0x30;
    cmd_buf[1] = 0x00;
    cmd_buf[2] = 0x00;
    cmd_buf[3] = 0x00;
    cmd_buf[4] = 0x00;
    cmd_buf[5] = 0x00;
    cmd_buf[6] = 0x00;
    cmd_buf[7] = 0x00;
    
    // Send message
    writeCmd(cmd_buf);
    delay(100);
    readBuf(cmd_buf);

    // Receive message
    command = reply_buf[0];
    CurrKP = reply_buf[2];
    CurrKI = reply_buf[3];
    SpdKP = reply_buf[4];
    SpdKI = reply_buf[5];
    PosKP = reply_buf[6];
    PosKI = reply_buf[7];
}

/** 
 * Write PID parametera to RAM command (one frame)
 * This command can write the parameters of current, speed, position loop KP and KI to RAM at 
 * one time, and it will not be saved after power off. The data type is uint8_t. The system 
 * sets the maximum range of PI parameters according to the motor model, and then divides it 
 * equally according to the maximum range of uint8_t of 256 units. Users only need to adjust 
 * 0-256 units.
 * The content of the reply data is the same as the sent data.
*/
void RMDArduinoUBUV360::writePIDRAM(uint8_t CurrKP, uint8_t CurrKI, uint8_t SpdKP, uint8_t SpdKI, uint8_t PosKP, uint8_t PosKI) {
    cmd_buf[0] = 0x31;
    cmd_buf[1] = 0x00;
    cmd_buf[2] = CurrKP;
    cmd_buf[3] = CurrKI;
    cmd_buf[4] = SpdKP;
    cmd_buf[5] = SpdKI;
    cmd_buf[6] = PosKP;
    cmd_buf[7] = PosKI;

    // Send message
    writeCmd(cmd_buf);
    readBuf(cmd_buf);

    // Receive message
    command = reply_buf[0];
    CurrKP = reply_buf[2];
    CurrKI = reply_buf[3];
    SpdKP = reply_buf[4];
    SpdKI = reply_buf[5];
    PosKP = reply_buf[6];
    PosKI = reply_buf[7];
}

/** 
 * Write PID parameters to ROM command (one frame)
 * This command can write the parameters of current, speed, position loop KP and KI to ROM at 
 * one time, which can be saved after power off. The data type is uint8_t. The system sets the 
 * maximum range of PI parameters according to the motor model, and then divides it equally 
 * according to the maximum range of uint8_t of 256 units. Users only need to adjust 0-256 units.
 * The content of the reply data is the same as the sent data.
*/
void RMDArduinoUBUV360::writePIDROM(uint8_t CurrKP, uint8_t CurrKI, uint8_t SpdKP, uint8_t SpdKI, uint8_t PosKP, uint8_t PosKI) {
    cmd_buf[0] = 0x32;
    cmd_buf[1] = 0x00;
    cmd_buf[2] = CurrKP;
    cmd_buf[3] = CurrKI;
    cmd_buf[4] = SpdKP;
    cmd_buf[5] = SpdKI;
    cmd_buf[6] = PosKP;
    cmd_buf[7] = PosKI;

    // Send message
    writeCmd(cmd_buf);
    readBuf(cmd_buf);

    // Receive message
    command = reply_buf[0];
    CurrKP = reply_buf[2];
    CurrKI = reply_buf[3];
    SpdKP = reply_buf[4];
    SpdKI = reply_buf[5];
    PosKP = reply_buf[6];
    PosKI = reply_buf[7];
}

/** 
 * Read acceleration data command (one frame)
 * The host sends this command to read the acceleration parameters of the current motor
 * The acceleration parameter is included in the drive response data. Acceleration data Accel is int32_t type, 
 * the unit is 1dps/s, and the parameter range is 50-60000.
*/
void RMDArduinoUBUV360::readAccel() { 
    cmd_buf[0] = 0x42;
    cmd_buf[1] = 0x00;
    cmd_buf[2] = 0x00;
    cmd_buf[3] = 0x00;
    cmd_buf[4] = 0x00;
    cmd_buf[5] = 0x00;
    cmd_buf[6] = 0x00;
    cmd_buf[7] = 0x00;

    // Send message
    writeCmd(cmd_buf);
    readBuf(cmd_buf);

    // Receive message
    command = reply_buf[0];
    Accel = ((uint32_t)reply_buf[7] << 24) + ((uint32_t)reply_buf[6] << 16) + ((uint32_t)reply_buf[5] << 8) + reply_buf[4];
}

/** 
 * Write acceleration to RAM and ROM command (one frame)
 * The host sends this command to write the acceleration and deceleration into RAM and ROM, which can be saved 
 * after power off. Acceleration data Accel is of uint32_t type, the unit is 1dps/s, and the parameter range is 
 * 100-60000. The command contains the acceleration and deceleration values in the position and velocity planning, 
 * which are determined by the index value. 
 * The motor will reply to the host after receiving the command, and the reply command is the same as the received 
 * command.
*/
void RMDArduinoUBUV360::writeAccel(uint8_t index, uint32_t Accel) { 
    cmd_buf[0] = 0x43;
    cmd_buf[1] = index & 0xFF;
    cmd_buf[2] = 0x00;
    cmd_buf[3] = 0x00;
    cmd_buf[4] = Accel & 0xFF;                       // 32 bit
    cmd_buf[5] = (Accel >> 8) & 0xFF;
    cmd_buf[6] = (Accel >> 16) & 0xFF;
    cmd_buf[7] = (Accel >> 24) & 0xFF;

    // Send message
    writeCmd(cmd_buf);
    readBuf(cmd_buf);

    // Receive message
    command = reply_buf[0];
    index = reply_buf[1];
    Accel = ((uint32_t)reply_buf[7] << 24) + ((uint32_t)reply_buf[6] << 16) + ((uint32_t)reply_buf[5] << 8) + reply_buf[4];
}

/** 
 * Read multi-turn encoder position data command
 * The host sends this command to read the multi-turn position of the encoder, which represents the rotation 
 * angle of the motor output shaft, including the multi-turn angle.
 * The motor replies to the host after receiving the command, and the frame data contains the following parameters. 
 * Encoder multi-turn position encoder (int32_t type, value range of multi-turn encoder, 4 bytes of valid data), 
 * which is the value after subtracting the encoder's multi-turn zero offset (initial position) from the original 
 * position of the encoder.
*/
void RMDArduinoUBUV360::readMultiTurnEncoderPos() { 
    cmd_buf[0] = 0x60;
    cmd_buf[1] = 0x00;
    cmd_buf[2] = 0x00;
    cmd_buf[3] = 0x00;
    cmd_buf[4] = 0x00;
    cmd_buf[5] = 0x00;
    cmd_buf[6] = 0x00;
    cmd_buf[7] = 0x00;

    // Send message
    writeCmd(cmd_buf);
    readBuf(cmd_buf);

    // Receive message
    command = reply_buf[0];
    encoder = ((uint32_t)reply_buf[7] << 24) + ((uint32_t)reply_buf[6] << 16) + ((uint32_t)reply_buf[5] << 8) + reply_buf[4];
}

/** 
 * Read multi-turn encoder original position data command 
 * without the zero offset (home position).
 * The motor replies to the host after receiving the command, and the frame data contains the following parameters. 
 * Encoder multi-turn raw position encoderRaw (int32_t type, value range, valid data 4 bytes).
*/
void RMDArduinoUBUV360::readMultiTurnOriginalPos() { 
    cmd_buf[0] = 0x61;
    cmd_buf[1] = 0x00;
    cmd_buf[2] = 0x00;
    cmd_buf[3] = 0x00;
    cmd_buf[4] = 0x00;
    cmd_buf[5] = 0x00;
    cmd_buf[6] = 0x00;
    cmd_buf[7] = 0x00;

    // Send message
    writeCmd(cmd_buf);
    readBuf(cmd_buf);

    // Receive message
    command = reply_buf[0];
    encoderRaw  = ((uint32_t)reply_buf[7] << 24) + ((uint32_t)reply_buf[6] << 16) + ((uint32_t)reply_buf[5] << 8) + reply_buf[4];
}

/** 
 * Read multi-turn encoder zero offset data command
 * The host sends this command to read the multi-turn zero offset value (initial position) of the encoder.
 * The motor replies to the host after receiving the command, and the frame data contains the following parameters. 
 * Encoder multi-turn zero offset encoderOffset (int32_t type, value range, valid data 4 bytes).
*/
void RMDArduinoUBUV360::readMultiTurnEncoderZeroOffset() { 
    cmd_buf[0] = 0x62;
    cmd_buf[1] = 0x00;
    cmd_buf[2] = 0x00;
    cmd_buf[3] = 0x00;
    cmd_buf[4] = 0x00;
    cmd_buf[5] = 0x00;
    cmd_buf[6] = 0x00;
    cmd_buf[7] = 0x00;

    // Send message
    writeCmd(cmd_buf);
    readBuf(cmd_buf);

    // Receive message
    command = reply_buf[0];
    encoderOffset = ((int32_t)reply_buf[7] << 24) + ((int32_t)reply_buf[6] << 16) + ((int32_t)reply_buf[5] << 8) + reply_buf[4];
}

/** 
 * Write encoder multi-turn value to ROM as motor zero command
 * The host sends this command to set the zero offset (initial position) of the encoder, where the encoder 
 * multi-turn value to be written, encoderOffset, is of type int32_t, (value range, 4 bytes of valid data).
 * Note: After writing the position of the new zero point, the motor needs to be restarted to be effective. 
 * Because of the change of the zero offset, the new zero offset (initial position) should be used as a reference 
 * when setting the target position.

*/
void RMDArduinoUBUV360::writeEncoderMultiTurnValueROMZero(int32_t encoderOffset) { 
    cmd_buf[0] = 0x63;
    cmd_buf[1] = 0x00;
    cmd_buf[2] = 0x00;
    cmd_buf[3] = 0x00;
    cmd_buf[4] = encoderOffset & 0xFF;                 // 32 bit
    cmd_buf[5] = (encoderOffset >> 8) & 0xFF;
    cmd_buf[6] = (encoderOffset >> 16) & 0xFF;
    cmd_buf[7] = (encoderOffset >> 24) & 0xFF;

    // Send message
    writeCmd(cmd_buf);
    readBuf(cmd_buf);

    // Receive message
    command = reply_buf[0];
    encoderOffset = ((int32_t)reply_buf[7] << 24) + ((int32_t)reply_buf[6] << 16) + ((int32_t)reply_buf[5] << 8) + reply_buf[4];
}


/** 
 * Write the current multi-turn position of the encoder to the ROM as the motor zero command
 * Write the current encoder position of the motor as the multi-turn encoder zero offset (initial position) into 
 * the ROM
 * Note: After writing the new zero point position, the motor needs to be restarted to be effective. Because of 
 * the change of the zero offset, the new zero offset (initial position) should be used as a reference when 
 * setting the target position.
 * The motor replies to the host after receiving the command, and the encoderOffset in the data is the set zero 
 * offset value.
*/
void RMDArduinoUBUV360::writeCurrentMultiTurnPositionROMZero() { 
    cmd_buf[0] = 0x64;
    cmd_buf[1] = 0x00;
    cmd_buf[2] = 0x00;
    cmd_buf[3] = 0x00;
    cmd_buf[4] = 0x00;
    cmd_buf[5] = 0x00;
    cmd_buf[6] = 0x00;
    cmd_buf[7] = 0x00;

    // Send message
    writeCmd(cmd_buf);
    readBuf(cmd_buf);

    // Receive message
    command = reply_buf[0];
    encoderOffset = ((int32_t)reply_buf[7] << 24) + ((int32_t)reply_buf[6] << 16) + ((int32_t)reply_buf[5] << 8) + reply_buf[4];
}

/** 
 * Read multi-turn angle command
 * The host sends this command to read the current multi-turn absolute angle value of the motor.
 * The motor replies to the host after receiving the command, and the frame data contains the following parameters.
 * 1. Motor angle motorAngle, (int32_t type, value range, valid data 4 bytes), unit 0.01º/LSB.

*/
void RMDArduinoUBUV360::readMultiTurnAngle() { 
    cmd_buf[0] = 0x92;
    cmd_buf[1] = 0x00;
    cmd_buf[2] = 0x00;
    cmd_buf[3] = 0x00;
    cmd_buf[4] = 0x00;
    cmd_buf[5] = 0x00;
    cmd_buf[6] = 0x00;
    cmd_buf[7] = 0x00;

    // Send message
    writeCmd(cmd_buf);
    readBuf(cmd_buf);

    // Receive message
    command = reply_buf[0];
    motorAngle = ((uint32_t)reply_buf[7] << 24) + ((uint32_t)reply_buf[6] << 16) + ((uint32_t)reply_buf[5] << 8) + reply_buf[4];
}

/** 
 * Read Motor Status 1 and Error Flag Command
 * This command reads the current motor temperature, voltage and error status flags
 * The motor replies to the host after receiving the command, and the frame data contains the following parameters:
 * 1. Motor temperature temperature (int8_t type, unit 1Cº/LSB).
 * 2. Brake control command: Indicates the state of the brake control command, 1 represents the brake release command, and 0 represents the brake lock command.
 * 2. Voltage (uint16_t type, unit 0.1V/LSB).
 * 3. Error flag errorState (of type uint16_t, each bit represents a different motor state)
*/
void RMDArduinoUBUV360::readMotorStatus1ErrorFlag() { 
    cmd_buf[0] = 0x9A;
    cmd_buf[1] = 0x00;
    cmd_buf[2] = 0x00;
    cmd_buf[3] = 0x00;
    cmd_buf[4] = 0x00;
    cmd_buf[5] = 0x00;
    cmd_buf[6] = 0x00;
    cmd_buf[7] = 0x00;

    // Send message
    writeCmd(cmd_buf);
    readBuf(cmd_buf);

    // Receive message
    command = reply_buf[0];
    temperature = reply_buf[1];
    RlyCtrlRslt = reply_buf[3];
    voltage = ((uint16_t)reply_buf[5] << 8) + reply_buf[4];
    errorState = ((uint16_t)reply_buf[7] << 8) + reply_buf[6];
}

/** 
 * Read Motor Status 2 Command
 * This command reads the temperature, speed and encoder position of the current motor.
 * The motor replies to the host after receiving the command, and the frame data contains the following parameters.
 * 1. Motor temperature temperature (int8_t type, 1Cº/LSB).
 * 2. The torque current value iq of the motor (int16_t type, 0.01A/LSB).
 * 3. Motor output shaft speed (int16_t type, 1dps/LSB).
 * 4. Motor output shaft angle (int16_t type, 1degree/LSB, maximum range +-32767degree).
*/
void RMDArduinoUBUV360::readMotorStatus2() { 
    cmd_buf[0] = 0x9C;
    cmd_buf[1] = 0x00;
    cmd_buf[2] = 0x00;
    cmd_buf[3] = 0x00;
    cmd_buf[4] = 0x00;
    cmd_buf[5] = 0x00;
    cmd_buf[6] = 0x00;
    cmd_buf[7] = 0x00;

    // Send message
    writeCmd(cmd_buf);
    readBuf(cmd_buf);

    // Receive message
    command = reply_buf[0];
    temperature = reply_buf[1];
    iq = ((uint16_t)reply_buf[3] << 8) + reply_buf[2];
    speed = ((uint16_t)reply_buf[5] << 8) + reply_buf[4];
    degree = ((uint16_t)reply_buf[7] << 8) + reply_buf[6];
}

/** 
 * Read Motor Status 3 Command
 * This command reads the current motor temperature and phase current data
 * The motor replies to the host after receiving the command, and the frame data contains the following data:
 * 1. Motor temperature temperature (int8_t type, 1Cº/LSB)
 * 2. Phase A current data, the data type is int16_t, and the corresponding actual phase current is 0.01ALSB.
 * 3.B-phase current data, the data type is int16_t type, and the corresponding actual phase current is 0.01ALSB.
 * 4. C-phase current data, the data type is int16_t type, and the corresponding actual phase current is 0.01ALSB.
*/
void RMDArduinoUBUV360::readMotorStatus3() { 
    cmd_buf[0] = 0x9D;
    cmd_buf[1] = 0x00;
    cmd_buf[2] = 0x00;
    cmd_buf[3] = 0x00;
    cmd_buf[4] = 0x00;
    cmd_buf[5] = 0x00;
    cmd_buf[6] = 0x00;
    cmd_buf[7] = 0x00;

    // Send message
    writeCmd(cmd_buf);
    readBuf(cmd_buf);

    // Receive message
    command = reply_buf[0];
    temperature = reply_buf[1];
    iA = ((uint16_t)reply_buf[3] << 8) + reply_buf[2];
    iB = ((uint16_t)reply_buf[5] << 8) + reply_buf[4];
    iC = ((uint16_t)reply_buf[7] << 8) + reply_buf[6];
}


/** 
 * Motor shutdown command
 * Turns off the motor output and also clears the motor running state, not in any closed loop mode.
 * The motor replies to the host after receiving the command, and the frame data is the same as that sent by the host.
*/
void RMDArduinoUBUV360::motorShutdown() { 
    cmd_buf[0] = 0x80;
    cmd_buf[1] = 0x00;
    cmd_buf[2] = 0x00;
    cmd_buf[3] = 0x00;
    cmd_buf[4] = 0x00;
    cmd_buf[5] = 0x00;
    cmd_buf[6] = 0x00;
    cmd_buf[7] = 0x00;

    writeCmd(cmd_buf);
    readBuf(cmd_buf);

    // Receive message
    command = reply_buf[0];
}

/** 
 * Motor stop command (one frame)
 * Stop the motor, the closed-loop mode where the motor is still running, just stop the motor speed.
 * The motor replies to the host after receiving the command, and the frame data is the same as that sent by the host
*/
void RMDArduinoUBUV360::stopMotor() {
    cmd_buf[0] = 0x81;
    cmd_buf[1] = 0x00;
    cmd_buf[2] = 0x00;
    cmd_buf[3] = 0x00;
    cmd_buf[4] = 0x00;
    cmd_buf[5] = 0x00;
    cmd_buf[6] = 0x00;
    cmd_buf[7] = 0x00;

    writeCmd(cmd_buf);
    readBuf(cmd_buf);

    // Receive message
    command = reply_buf[0];
}


/**
 * Torque closed-loop control command
 * This command is a control command, which can be run when the motor is not faulty. The host sends this command 
 * to control the torque and current output of the motor. The control value iqControl is of type int16_t and the 
 * unit is 0.01A/LSB.
 * The motor replies to the host after receiving the command, and the frame data contains the following parameters.
 * 5. Motor temperature temperature (int8_t type, 1Cº/LSB).
 * 6. The torque current value iq of the motor (int16_t type, 0.01A/LSB).
 * 7. Motor output shaft speed (int16_t type, 1dps/LSB).
 * 8. Motor output shaft angle (int16_t type, 1degree/LSB, maximum range +132767degree).
 */
void RMDArduinoUBUV360::torqueControl(int16_t iqControl) { 
    cmd_buf[0] = 0xA1;
    cmd_buf[1] = 0x00;
    cmd_buf[2] = 0x00;
    cmd_buf[3] = 0x00;
    cmd_buf[4] = iqControl & 0xFF;                  // 16 bit
    cmd_buf[5] = (iqControl) >> 8 & 0xFF;
    cmd_buf[6] = 0x00;
    cmd_buf[7] = 0x00;

    // Send message
    writeCmd(cmd_buf);
    readBuf(cmd_buf);

    // Receive message
    command = reply_buf[0];
    temperature = reply_buf[1];
    iq = ((int16_t)reply_buf[3] << 8) + reply_buf[2];
    speed = ((int16_t)reply_buf[5] << 8) + reply_buf[4];
    degree = ((int16_t)reply_buf[7] << 8) + reply_buf[6];
}

/**
 * Speed Closed-loop Control Command
 * This command is a control command, which can be run when the motor is not faulty. The host sends this command 
 * to control the speed of the motor output shaft. The control value speedControl is int32_t type, and the 
 * corresponding actual speed is 0.01dps/LSB.
 * The motor replies to the host after receiving the command, and the frame data contains the following parameters.
 * 1. Motor temperature temperature (int8_t type, 1Cº/LSB).
 * 2. The torque current value iq of the motor (int16_t type, 0.01A/LSB).
 * 3. Motor output shaft speed (int16_t type, 1dps/LSB).
 * 4. Motor output shaft angle (int16_t type, 1degree/LSB, maximum range +-32767degree).
 */
void RMDArduinoUBUV360::speedClosedLoopControl(int32_t speedControl) { 
    cmd_buf[0] = 0xA2;
    cmd_buf[1] = 0x00;
    cmd_buf[2] = 0x00;
    cmd_buf[3] = 0x00;
    cmd_buf[4] = speedControl & 0xFF;                // 32 bit
    cmd_buf[5] = (speedControl >> 8) & 0xFF;
    cmd_buf[6] = (speedControl >> 16) & 0xFF;
    cmd_buf[7] = (speedControl >> 24) & 0xFF;

    // Send message
    writeCmd(cmd_buf);
    readBuf(cmd_buf);

    // Receive message
    command = reply_buf[0];
    temperature = reply_buf[1];
    iq = ((int16_t)reply_buf[3] << 8) + reply_buf[2];
    speed = ((int16_t)reply_buf[5] << 8) + reply_buf[4];
    degree = ((int16_t)reply_buf[7] << 8) + reply_buf[6];
}

/**
 * Position tracking control command
 * This command is a control command, which can be run when the motor is not faulty. The host sends this command 
 * to control the position of the motor (multi-turn angle). The control value angleControl is int32_t type, and 
 * the corresponding actual position is 0.01degree/LSB, that is, 36000 represents 360º, and the rotation direction 
 * of the motor is determined by the difference between the target position and the current position . The A3 
 * command is used for direct position tracking. After the motor receives the target position, it is compared 
 * with the current position and then output to the subsequent stage after passing through the PI controller.
 * If the customer has realized the trajectory planning in the control, it can be achieved by sending different 
 * target positions in a fixed cycle through the A3 command. At the same time, the speed and acceleration of the 
 * motor during the movement process also depend on the speed and acceleration of the trajectory planning.
 * The motor replies to the host after receiving the command, and the frame data contains the following parameters.
 * 1. Motor temperature temperature (int8_t type, 1Cº/LSB).
 * 2. The torque current value iq of the motor (int16_t type, 0.01A/LSB).
 * 3. Motor output shaft speed (int16_t type, 1dps/LSB).
 * 4. Motor output shaft angle (int16_t type, 1degree/LSB, maximum range +-32767degree).

 */
void RMDArduinoUBUV360::positionTrackingControl(int32_t angleControl) { 
    cmd_buf[0] = 0xA3;
    cmd_buf[1] = 0x00;
    cmd_buf[2] = 0x00;
    cmd_buf[3] = 0x00;
    cmd_buf[4] = angleControl & 0xFF; 		// 32 bits
    cmd_buf[5] = (angleControl >> 8) & 0xFF;
    cmd_buf[6] = (angleControl >> 16) & 0xFF;
    cmd_buf[7] = (angleControl >> 24) & 0xFF;

    // Send message
    writeCmd(cmd_buf);
    readBuf(cmd_buf);

    // Receive message
    command = reply_buf[0];
    temperature = reply_buf[1];
    iq = ((int16_t)reply_buf[3] << 8) + reply_buf[2];
    speed = ((int16_t)reply_buf[5] << 8) + reply_buf[4];
    degree = ((uint16_t)reply_buf[7] << 8) + reply_buf[6];
}

/**
 * Absolute position closed-loop control command
 * This command is a control command, which can be run when the motor is not faulty. The host sends this command 
 * to control the position of the motor (multi-turn angle). The control value angleControl is int32_t type, and 
 * the corresponding actual position is 0.01degree/LSB, that is, 36000 represents 360º, and the rotation direction 
 * of the motor is determined by the difference between the target position and the current position . The control 
 * value maxSpeed limits the maximum speed of the motor output shaft rotation, which is of type uint16_t, 
 * corresponding to the actual speed of 1dps/LSB.
 * The motor replies to the host after receiving the command, and the frame data contains the following parameters.
 * 5. Motor temperature temperature (int8_t type, 1Cº/LSB).
 * 6. The torque current value iq of the motor (int16_t type, 0.01A/LSB).
 * 7. Motor output shaft speed (int16_t type, 1dps/LSB).
 * 8. Motor output shaft angle (int16_t type, 1degree/LSB, maximum range +-32767degree).
 */
void RMDArduinoUBUV360::absolutePositionControl(uint16_t maxSpeed, int32_t angleControl) { 
    cmd_buf[0] = 0xA4;
    cmd_buf[1] = 0x00;
    cmd_buf[2] = maxSpeed & 0xFF;               // 16
    cmd_buf[3] = (maxSpeed >> 8) & 0xFF;
    cmd_buf[4] = angleControl & 0xFF; 		// 32 bits
    cmd_buf[5] = (angleControl >> 8) & 0xFF;
    cmd_buf[6] = (angleControl >> 16) & 0xFF;
    cmd_buf[7] = (angleControl >> 24) & 0xFF;

    // Send message
    writeCmd(cmd_buf);
    readBuf(cmd_buf);

    // Receive message
    command = reply_buf[0];
    temperature = reply_buf[1];
    iq = ((int16_t)reply_buf[3] << 8) + reply_buf[2];
    speed = ((int16_t)reply_buf[5] << 8) + reply_buf[4];
    degree = ((uint16_t)reply_buf[7] << 8) + reply_buf[6];
}

/**
 * Position tracking control command with speed limit 
 * This command is a control command, which can be run when the motor is not faulty. The host sends this command 
 * to control the position of the motor (multi-turn angle). The control value angleControl is int32_t type, and 
 * the corresponding actual position is 0.01degree/LSB, that is, 36000 represents 360º, and the rotation direction 
 * of the motor is determined by the difference between the target position and the current position . The control 
 * value maxSpeed limits the maximum speed of the motor output shaft rotation, which is of type uint16_t, 
 * corresponding to the actual speed of 1dps/LSB. The A5 command is used for direct position tracking. After the 
 * motor receives the target position, it is compared with the current position and then output to the subsequent 
 * stage after passing through the PI controller.
 * If the customer has realized the trajectory planning in the control, it can be achieved by sending different 
 * target positions in a fixed cycle through the A5 command. At the same time, the speed of the motor during the 
 * movement is limited by the set maximum speed.
 * The motor replies to the host after receiving the command, and the frame data contains the following parameters.
 * 9. Motor temperature temperature (int8_t type, 1Cº/LSB).
 * 10. The torque current value iq of the motor (int16_t type, 0.01A/LSB).
 * 11. Motor output shaft speed (int16_t type, 1dps/LSB).
 * Motor output shaft angle (int16_t type, 1degree/LSB, maximum range +-32767degree).
 * 12. Motor output shaft angle (int16_t type, 1degree/LSB, maximum range +-32767degree).
 */
void RMDArduinoUBUV360::positionControlSpeedLimit(uint16_t maxSpeed, int32_t angleControl) { 
    cmd_buf[0] = 0xA5;
    cmd_buf[1] = 0x00;
    cmd_buf[2] = maxSpeed & 0xFF;               // 16
    cmd_buf[3] = (maxSpeed >> 8) & 0xFF;
    cmd_buf[4] = angleControl & 0xFF; 		// 32 bits
    cmd_buf[5] = (angleControl >> 8) & 0xFF;
    cmd_buf[6] = (angleControl >> 16) & 0xFF;
    cmd_buf[7] = (angleControl >> 24) & 0xFF;

    // Send message
    writeCmd(cmd_buf);
    readBuf(cmd_buf);

    // Receive message
    command = reply_buf[0];
    temperature = reply_buf[1];
    iq = ((int16_t)reply_buf[3] << 8) + reply_buf[2];
    speed = ((int16_t)reply_buf[5] << 8) + reply_buf[4];
    degree = ((uint16_t)reply_buf[7] << 8) + reply_buf[6];
}

/** 
 * Incremental position closed-loop control command
 * This command is a control command, which can be run when the motor is not faulty. The host sends this command 
 * to control the incremental position (multi-turn angle) of the motor, and run the input position increment with 
 * the current position as the starting point. The control value angleControl is of type int32_t, and the 
 * corresponding actual position is 0.01degree/LSB, that is, 36000 represents 360°, and the rotation direction of 
 * the motor is determined by the incremental position symbol.
 * The control value maxSpeed limits the maximum speed of the motor output shaft rotation, which is of type 
 * uint16_t, corresponding to the actual speed of 1dps/LSB. 
 * The motor replies to the host after receiving the command, and the frame data contains the following parameters.
 * 5. Motor temperature temperature (int8_t type, 1Cº/LSB).
 * 6. The torque current value iq of the motor (int16_t type, 0.01A/LSB).
 * 7. Motor output shaft speed (int16_t type, 1dps/LSB).
 * 8. Motor output shaft angle (int16_t type, 1degree/LSB, maximum range +-32767degree).
 */
void RMDArduinoUBUV360::incrementalPositionControl(uint16_t maxSpeed, int32_t angleControl) { 
    cmd_buf[0] = 0xA8;
    cmd_buf[1] = temperature;
    cmd_buf[2] = maxSpeed & 0xFF;               // 16
    cmd_buf[3] = (maxSpeed >> 8) & 0xFF;
    cmd_buf[4] = angleControl & 0xFF; 		// 32 bits
    cmd_buf[5] = (angleControl >> 8) & 0xFF;
    cmd_buf[6] = (angleControl >> 16) & 0xFF;
    cmd_buf[7] = (angleControl >> 24) & 0xFF;

    // Send message
    writeCmd(cmd_buf);
    readBuf(cmd_buf);

    // Receive message
    command = reply_buf[0];
    temperature = reply_buf[1];
    iq = ((int16_t)reply_buf[3] << 8) + reply_buf[2];
    speed = ((int16_t)reply_buf[5] << 8) + reply_buf[4];
    degree = ((uint16_t)reply_buf[7] << 8) + reply_buf[6];
}

/** 
 * System operating mode acquisition command
 * This command reads the current motor running mode.
 * The motor replies to the host after receiving the command, and the drive reply data contains the running state of the parameter runmode, which is of type uint8_t.
 * The motor operation mode has the following 4 states:
 * 1. Current loop mode (0x01).
 * 2. Speed loop mode (0x02).
 * 3. Position loop mode (0x03).
 */
void RMDArduinoUBUV360::systemOperatingModeAcquisition() { 
    cmd_buf[0] = 0x70;
    cmd_buf[1] = 0x00;
    cmd_buf[2] = 0x00;
    cmd_buf[3] = 0x00;
    cmd_buf[4] = 0x00;
    cmd_buf[5] = 0x00;
    cmd_buf[6] = 0x00;
    cmd_buf[7] = 0x00;

    // Send message
    writeCmd(cmd_buf);
    readBuf(cmd_buf);

    // Receive message
    command = reply_buf[0];
    runmode = reply_buf[7];
}

/** 
 * Motor power acquisition command
 * This command reads the current motor power.
 * The motor replies to the host after receiving the command, and the drive reply data contains the motor power 
 * parameter motorpower, which is of type uint16_t, the unit is watt, and the unit is 0.1w/LSB.
 */
void RMDArduinoUBUV360::motorPowerAcquisition() { 
    cmd_buf[0] = 0x71;
    cmd_buf[1] = 0x00;
    cmd_buf[2] = 0x00;
    cmd_buf[3] = 0x00;
    cmd_buf[4] = 0x00;
    cmd_buf[5] = 0x00;
    cmd_buf[6] = 0x00;
    cmd_buf[7] = 0x00;

    // Send message
    writeCmd(cmd_buf);
    readBuf(cmd_buf);

    // Receive message
    command = reply_buf[0];
    motorpower = ((uint16_t)reply_buf[7] << 8) + reply_buf[6];
}

/** 
 * System reset command
 * This command is used to reset the system program.
 * The motor will reset after receiving the command and will not return to the command.
 */
void RMDArduinoUBUV360::systemReset() { 
    cmd_buf[0] = 0x76;
    cmd_buf[1] = 0x00;
    cmd_buf[2] = 0x00;
    cmd_buf[3] = 0x00;
    cmd_buf[4] = 0x00;
    cmd_buf[5] = 0x00;
    cmd_buf[6] = 0x00;
    cmd_buf[7] = 0x00;

    // Send message
    writeCmd(cmd_buf);
    readBuf(cmd_buf);

    // Receive message
    command = reply_buf[0];
}

/** 
 * System brake release command
 * This command is used to reset the system program.
 * The motor will reset after receiving the command and will not return to the command.
 */
void RMDArduinoUBUV360::systemBrakeRelease() { 
    cmd_buf[0] = 0x77;
    cmd_buf[1] = 0x00;
    cmd_buf[2] = 0x00;
    cmd_buf[3] = 0x00;
    cmd_buf[4] = 0x00;
    cmd_buf[5] = 0x00;
    cmd_buf[6] = 0x00;
    cmd_buf[7] = 0x00;

    // Send message
    writeCmd(cmd_buf);
    readBuf(cmd_buf);

    // Receive message
    command = reply_buf[0];
}

/** 
 * System brake lock command
 * This command is used to close the system holding brake. The holding brake locks the motor and the motor can no 
 * longer run. The holding brake is also in this state after the system is powered off.
 * The motor replies to the host after receiving the command, and the frame data is the same as the command sent 
 * by the host.
 */
void RMDArduinoUBUV360::systemBrakeLock() { 
    cmd_buf[0] = 0x78;
    cmd_buf[1] = 0x00;
    cmd_buf[2] = 0x00;
    cmd_buf[3] = 0x00;
    cmd_buf[4] = 0x00;
    cmd_buf[5] = 0x00;
    cmd_buf[6] = 0x00;
    cmd_buf[7] = 0x00;

    // Send message
    writeCmd(cmd_buf);
    readBuf(cmd_buf);

    // Receive message
    command = reply_buf[0];
}

/** 
 * System runtime read command
 * This command is used to obtain the system running time in ms.
 * The motor replies to the host after receiving the command, and the drive reply data contains the system running 
 * time SysRunTime, which is uint32_t type, and the unit is ms.
 */
void RMDArduinoUBUV360::systemRuntimeRead() { 
    cmd_buf[0] = 0xB1;
    cmd_buf[1] = 0x00;
    cmd_buf[2] = 0x00;
    cmd_buf[3] = 0x00;
    cmd_buf[4] = 0x00;
    cmd_buf[5] = 0x00;
    cmd_buf[6] = 0x00;
    cmd_buf[7] = 0x00;

    // Send message
    writeCmd(cmd_buf);
    readBuf(cmd_buf);

    // Receive message
    command = reply_buf[0];
    SysRunTime = ((uint32_t)reply_buf[7] << 24) + ((uint32_t)reply_buf[6] << 16) + ((uint32_t)reply_buf[5] << 8) + reply_buf[4];
}

/** 
 * System software version date read command
 * This command is used to get the update date of the system software version.
 * The motor will reply to the host after receiving the command. The driver reply data contains the latest version 
 * date of the system software, VersionDate, which is of type uint32_t. The date format is in the format of year, 
 * month, and day, such as 20211126.
 */
void RMDArduinoUBUV360::systemSoftwareVersionDateRead() { 
    cmd_buf[0] = 0xB2;
    cmd_buf[1] = 0x00;
    cmd_buf[2] = 0x00;
    cmd_buf[3] = 0x00;
    cmd_buf[4] = 0x00;
    cmd_buf[5] = 0x00;
    cmd_buf[6] = 0x00;
    cmd_buf[7] = 0x00;

    // Send message
    writeCmd(cmd_buf);
    readBuf(cmd_buf);

    // Receive message
    command = reply_buf[0];
    VersionDate = ((uint32_t)reply_buf[7] << 24) + ((uint32_t)reply_buf[6] << 16) + ((uint32_t)reply_buf[5] << 8) + reply_buf[4];
}

/** 
 * Communication interruption protection time setting command
 * This command is used to set the communication interruption protection time in ms. If the communication is 
 * interrupted for more than the set time, it will cut off the output brake lock. To run again, you need to 
 * establish stable and continuous communication first. Writing 0 means that the communication interruption 
 * protection function is not enabled.
 * The motor replies to the host after receiving the command, and the frame data is the same as the command sent 
 * by the host.
 */
void RMDArduinoUBUV360::communicationInterruptionProtectionTimeSetting(uint32_t CanRecvTime_MS) { 
    cmd_buf[0] = 0xB3;
    cmd_buf[1] = 0x00;
    cmd_buf[2] = 0x00;
    cmd_buf[3] = 0x00;
    cmd_buf[4] = CanRecvTime_MS & 0xFF; 		// 32 bits
    cmd_buf[5] = (CanRecvTime_MS >> 8) & 0xFF;
    cmd_buf[6] = (CanRecvTime_MS >> 16) & 0xFF;
    cmd_buf[7] = (CanRecvTime_MS >> 24) & 0xFF;

    // Send message
    writeCmd(cmd_buf);
    readBuf(cmd_buf);

    // Receive message
    command = reply_buf[0];
    CanRecvTime_MS = ((uint32_t)reply_buf[7] << 24) + ((uint32_t)reply_buf[6] << 16) + ((uint32_t)reply_buf[5] << 8) + reply_buf[4];
}

/** 
 * Communication baud rate setting command
 * This instruction can set the communication baudrate of CAN and RS485 bus. The parameters will be saved in ROM 
 * after setting, and will be saved after power off, and will run at the modified baudrate when powered on again.
 * Since the communication baud rate is modified, the reply command is random and need not be processed.
 */
void RMDArduinoUBUV360::communicationbaudRateSetting(uint8_t baudrate) { 
    cmd_buf[0] = 0xB4;
    cmd_buf[1] = 0x00;
    cmd_buf[2] = 0x00;
    cmd_buf[3] = 0x00;
    cmd_buf[4] = 0x00; 		
    cmd_buf[5] = 0x00;
    cmd_buf[6] = 0x00;
    cmd_buf[7] = baudrate & 0xFF;

    // Send message
    writeCmd(cmd_buf);
    readBuf(cmd_buf);

    // Receive message
    command = reply_buf[0];
    baudrate = reply_buf[7];
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
// unsigned char da el equivalente de código ASCII (Ej:  97=a) ?????? Hay que comprobar que funciona bien.
//////////////////////////////////////////////////////////////////////////////////////////////////////
/** 
 * Motor model reading command
 * This command is used to read the motor model, and the read data is ACSII code, which can be converted into the 
 * corresponding actual symbol by checking the ACSII code table.
 */
void RMDArduinoUBUV360::motorModelReading() { 
    cmd_buf[0] = 0xB5;
    cmd_buf[1] = 0x00;
    cmd_buf[2] = 0x00;
    cmd_buf[3] = 0x00;
    cmd_buf[4] = 0x00; 		
    cmd_buf[5] = 0x00;
    cmd_buf[6] = 0x00;
    cmd_buf[7] = 0x00;

    // Send message
    writeCmd(cmd_buf);
    readBuf(cmd_buf);

    // Receive message
    command = reply_buf[0];
    Type1 = reply_buf[1];
    Type2 = reply_buf[2];
    Type3 = reply_buf[3];
    Type4 = reply_buf[4];
    Type5 = reply_buf[5];
    Type6 = reply_buf[6];
    Type7 = reply_buf[7];
}


/** 
 * Function control command
 * This instruction is used to use some specific functions. It is a compound function instruction, which can 
 * contain multiple function control instructions.
 * The motor replies to the host computer after receiving the command, and the frame data is the same as the 
 * command sent by the host computer.
 */
void RMDArduinoUBUV360::functionControl(uint8_t index, uint32_t Value) { 
    cmd_buf[0] = 0x20;
    cmd_buf[1] = index & 0xFF;
    cmd_buf[2] = 0x00;
    cmd_buf[3] = 0x00;
    cmd_buf[4] = Value & 0xFF;                 //32
    cmd_buf[5] = (Value >> 8) & 0xFF;
    cmd_buf[6] = (Value >> 16) & 0xFF;
    cmd_buf[7] = (Value >> 24) & 0xFF;

    // Send message
    writeCmd(cmd_buf);
    readBuf(cmd_buf);

    // Receive message
    command = reply_buf[0];
    index = reply_buf[1];
    CanRecvTime_MS = ((uint32_t)reply_buf[7] << 24) + ((uint32_t)reply_buf[6] << 16) + ((uint32_t)reply_buf[5] << 8) + reply_buf[4];
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
// ???????????
// No está bien definido porque no se ha metido el comando a ejecutar de ninguna forma. De hecho, no encuentro variable alguna que lo contenga.
// Hay que probar si la repuesta modifica la variable MOTOR_ADDRESS porque cuando se manda contiene 0x280 como dirección 
// destino, pero cundo se lee la respuesta, debería contener la dirección de un motor tras otro. 
// Paso a paso. Primero que muestre la primera dirección y después se hará un bucle para que muestre cada respuesta.
// Faltaría definir qué hacer con las respuestas. Pero eso depende de cada comando enviado y se hará cuando se pruebe que funciona el envío.
//////////////////////////////////////////////////////////////////////////////////////////////////////
/** 
 * Multi-motor command + COMMAND
 * The ID number is 280, which means that multiple motors correspond to the same command at the same time. The 
 * content and function of the instruction are the same as those of the single-motor instruction. For details, 
 * please refer to the single-motor instruction.
 */
void RMDArduinoUBUV360::multiMotor(uint8_t command) { // 0x280
    cmd_buf[0] = command & 0xFF; // Dependiendo de command, el resto de bytes tendrá el significado de la respuesta de ese comando.
    cmd_buf[1] = 0x00;
    cmd_buf[2] = 0x00;
    cmd_buf[3] = 0x00;
    cmd_buf[4] = 0x00;
    cmd_buf[5] = 0x00;
    cmd_buf[6] = 0x00;
    cmd_buf[7] = 0x00;
    MOTOR_ADDRESS = 0x280;

    // Send message
    writeCmd(cmd_buf);
    readBuf(cmd_buf);

    // Receive message 
    // Se identifica a cada motor que responde por la dirección MOTOR_ADDRESS = 0x241, 0x242, 0z243, ... (Direcciones de cada motor que responda + COMMAND enviado)
    command = reply_buf[0]; // COMMAND (Direcciones de cada motor que responda + COMMAND enviado)
    //readBuf(cmd_buf);
    // Receive message 
    // Se identifica a cada motor que responde por la dirección MOTOR_ADDRESS = 0x241, 0x242, 0z243, ... (Direcciones de cada motor que responda + COMMAND enviado)
    // Receive message
    //command = reply_buf[0];
    //temperature = reply_buf[1];
    //iq = ((int16_t)reply_buf[3] << 8) + reply_buf[2];
    //speed = ((int16_t)reply_buf[5] << 8) + reply_buf[4];
    //encoder = ((uint16_t)reply_buf[7] << 8) + reply_buf[6]; // Cuidado porque hay un encoder definido como uint32_t
    CAN_MOTOR_ADDRESS = _CAN.getCanId(); // Se extrae la ID del motor que responda en primer lugar. Para el resto se deben plantear interrupciones. 
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
//?????????? 
// El tipo bool de la variable wReadWriteFlag creo que está bien y que está incluido en mcp_can.h-cpp 
// No estoy seguro de la definición de CANID, pero creo que está bien
//////////////////////////////////////////////////////////////////////////////////////////////////////
/** 
 * CANID setting command
 * This command is used to set and read CAN ID.
 * The host sends this command to set and read the CAN ID, the parameters are as follows.
 * 1. The read and write flag bit wReadWriteFlag is bool type, 1 read 0 write.
 * 2. CANID, size range (#1~#32), uint16_t type (synchronized with the upper computer function), device identifier 0x140 + ID (1~32).
 * 
 * 1. The motor replies to the host after receiving the command, which is divided into the following two situations:
 * 2. Set CANID, the range is 1-32, and return to the original command.
 * 3. Read CANID, the return parameters are as follows.
 */
void RMDArduinoUBUV360::CANIDSetting(bool wReadWriteFlag, uint16_t CANID) { 
    cmd_buf[0] = 0x79;
    cmd_buf[1] = 0x00;
    cmd_buf[2] = wReadWriteFlag & 0xFF;
    cmd_buf[3] = 0x00;
    cmd_buf[4] = 0x00;
    cmd_buf[5] = 0x00;
    cmd_buf[6] = 0x00;
    cmd_buf[7] = CANID & 0xFF;

    // Send message
    writeCmd(cmd_buf);
    readBuf(cmd_buf);

    // Receive message
    command = reply_buf[0]; 
    wReadWriteFlag = reply_buf[2];
    CANID = ((uint16_t)reply_buf[7] << 8) + reply_buf[6];
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
//??????   
// Creo que está mal definido en el pdf proque mete en el primer byte la parte alta de la variable, y en el segundo, la parte baja. Pero, luego, en la parte alta, dice que 
// hay que meter la parte baja antes que la alta y en el segundo byte, lo mismo. Traduciendo: "15 14 13 12" "11 10 9 8" "7 6 5 4" "3 2 1 0" (6 E 5 6) pasan a estar 
// "11 10 9 8" "15 14 13 12" "3 2 1 0" "7 6 5 4". Y al unirlo en el ejemplo para traducirlo dice que lo une tal como lo ha metido (E 6 6 5).  Luego la variable inicial no es como 
// la final. Por tanto, mientras lo resuelven lo voy a definir como me parece a la falta de probarlo proque sospecho que el ejemplo es lo uqe querían obtener. Luego, primero meteré 
// la parte alta sin permutar los dos cuartetos de bits y lo mismo en el segundo byte de la parte baja. 
// Y con la respuesta pasa lo mismo. Lo que mete teóricamente en la respueta no es lo mismo que pone en el ejemplo. 
// Quiero pensar que con el ejemplo entiende lo que quiere mandar y recibir pero que al definir los cartetos de bits se ha equivocado en la posición dentro de los bytes correspondientes...
//////////////////////////////////////////////////////////////////////////////////////////////////////
/** 
 * Motion Mode Control Command_CAN + ID
 * The command consists of 5 input parameters:
 *   p_des (desired position),
 *   v_des(desired velocity),
 *   t_ff (feedforward torque),
 *   kp (position deviation coefficient),
 *   kd (speed deviation coefficient).
 *   Each parameter has a preset range size:
 *   p_des: -12.5 to 12.5 in rad;
 *   v_des: -45 to 45, in rad/s;
 *   t_ff: -24 to 24, unit N-m;
 *   kp: 0 to 500;
 *   kd: 0 to 5;
 * Function expression: IqRef = [kp*(p_des - p_fd_actual position) + kd*(v_des - v_fb_actual speed) + 
 * t_ff]*KT_torque coefficient; IqRef is the output current of the last given motor.
 */
void RMDArduinoUBUV360::motionModeControlCAN(uint16_t p_des, uint16_t v_des, uint16_t kp, uint16_t kd, uint16_t t_ff) { // 0x400;
    cmd_buf[0] = (p_des >> 8) & 0xFF; // Desplaza p_des 8 bits hacia la derecha y mete los siguientes 8 bits (High).
    cmd_buf[1] = p_des & 0xFF; // Mete los 8 primeros bits de la derecha de p_des (Low).
    cmd_buf[2] = (v_des >> 4) & 0xFF; // Desplaza v_des 4 posiciones hacia la derecha y mete los siguientes 8 bits restantes (High). 
    cmd_buf[3] = (v_des & 0xF) + ((kp >> 8) & 0xF); // Mete los primeros 4 bits de la derecha de v_des (Low). Y desplaza kp 8 bits a la derecha y mete los 4 bits restantes (High).
    cmd_buf[4] = kp & 0xFF; // Mete los 8 primeros bits de la derecha de kp (Low).
    cmd_buf[5] = (kd >> 4) & 0xFF; // Desplaza kd 4 espacios a la derecha y mete los 8 bits restantes de kd (High). 
    cmd_buf[6] = (kd & 0xF) + ((t_ff >> 8) & 0xF); // Mete los primeros 4 bits de la derecha de kd (Low). Y desplaza t_ff 8 bits a la derecha y mete los 4 bits restantes (High).
    cmd_buf[7] = t_ff & 0xFF; // Mete los primeros 8 bits (Low).

    // Send message
    writeCmd(cmd_buf);
    readBuf(cmd_buf);

    // Receive message
    //command = reply_buf[0]; 
    p_des = ((uint16_t)reply_buf[0] << 8) + reply_buf[1]; // Para unirlos se hace desplazando la parte high hacia la izquierda y sumando la parte baja en la derecha. De diferente forma a como se ha hecho hasta esta función.
    v_des = ((uint16_t)reply_buf[2] << 4) + ((reply_buf[3] >> 4) & 0xF);
    kp = (((uint16_t)reply_buf[3] & 0xF) << 8) + reply_buf[4]; // LowByte(reply_buf[3]) es similar a (uint12_t)reply_buf[3] & 0xF
    kd = ((uint16_t)reply_buf[5] << 4) + ((reply_buf[6] >> 4) & 0xF); // HighByte(reply_buf[6]) es similar a ((reply_buf[6] >> 4) & 0xF)
    t_ff = (((uint16_t)reply_buf[6] & 0xF) << 8)  + reply_buf[7];
}

//////////////////////////////////////////////////////////////////////////////////////////////////////

// General function
void RMDArduinoUBUV360::serialWriteTerminator() {
    Serial.write(13);
    Serial.write(10);
}

// Private
void RMDArduinoUBUV360::readBuf(unsigned char *buf) {
    delayMicroseconds(600);    // 1000us
    if (CAN_MSGAVAIL == _CAN.checkReceive()) {
        _CAN.readMsgBuf(&len, tmp_buf);
        CAN_MOTOR_ADDRESS = _CAN.getCanId(); // Se extrae la ID del motor que responda en primer lugar. Para el resto se deben plantear interrupciones. 
        if (tmp_buf[0] == buf[0]) {
            reply_buf[0] = tmp_buf[0];
            reply_buf[1] = tmp_buf[1];
            reply_buf[2] = tmp_buf[2];
            reply_buf[3] = tmp_buf[3];
            reply_buf[4] = tmp_buf[4];
            reply_buf[5] = tmp_buf[5];
            reply_buf[6] = tmp_buf[6];
            reply_buf[7] = tmp_buf[7];
        }
    }
}

void RMDArduinoUBUV360::writeCmd(unsigned char *buf) {
    // CAN 
    unsigned char sendState = _CAN.sendMsgBuf(MOTOR_ADDRESS, 0, 8, buf);
    if (sendState != CAN_OK) { //  //
        Serial.println("Error Sending Message (RMDArduinoUBUV360.h)...");
        Serial.print("sendState= "); Serial.println(sendState);
        Serial.print(" - CAN_FAILTX= ");Serial.print(CAN_FAILTX);
        Serial.print(" - CAN_OK= ");Serial.println(CAN_OK);
    }
}
