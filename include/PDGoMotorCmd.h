#pragma once

#include "PDLog.h"
#include "PDGoMotorCRC.h"

struct PDGoMotorCmd {
    enum Mode {
        BRAKE = 0,
        FOC = 1
    };

    PDGoMotorCmd() {
        memset(fBytes, '\0', sizeof(fBytes));
        fHeader[0] = 0xFE;
        fHeader[1] = 0xEE;
    }

    static constexpr float GEAR_RATIO = 6.33;

    inline Mode getMode() const {
        return Mode((cmd.fModeID>>4)&0xF);
    }

    inline void setMode(Mode mode) {
        cmd.fModeID = ((cmd.fModeID&0x0F) | ((uint8_t(mode)&0xF) << 4));
    }

    inline void setBootMode() {
        cmd.fModeID = 0x7F;

    }

    inline void setBrakeMode() {
        setMode(BRAKE);
    }

    inline void setFOCMode() {
        setMode(FOC);
    }

    inline uint8_t getMotorID() const {
        return (cmd.fModeID&0xF);
    }

    inline void setMotorID(uint8_t motorID) {
        cmd.fModeID = ((cmd.fModeID&0xF0) | (motorID&0xF));
        fValid = true;
    }

    inline void setTau(float tau) {
        int16_t tau_int = int16_t(tau*256);
        cmd.fTau[0] = uint8_t((tau_int>>0)&0xFF);
        cmd.fTau[1] = uint8_t((tau_int>>8)&0xFF);
    }

    inline void setDQ(float dq) {
        if (getMode() == FOC) {
            int16_t dq_int = int16_t(dq/25.6*32768.0);
            cmd.fDQ[0] = uint8_t((dq_int>>0)&0xFF);
            cmd.fDQ[1] = uint8_t((dq_int>>8)&0xFF);
        } else {
            cmd.fDQ[0] = 0;
            cmd.fDQ[1] = 0;
        }
    }

    inline void setQ(float q) {
        if (q >= 411774) {

        } else if (q <= -411774) {

        } else {
            int32_t q_int = int32_t(q/6.2832*32768.0);
            cmd.fQ[0] = uint8_t((q_int>>0)&0xFF);
            cmd.fQ[1] = uint8_t((q_int>>8)&0xFF);
            cmd.fQ[2] = uint8_t((q_int>>16)&0xFF);
            cmd.fQ[3] = uint8_t((q_int>>24)&0xFF);
        }
    }

    inline void setQRadians(float radians) {
        setQ(radians * GEAR_RATIO);
    }

    inline void setQDegrees(float degrees) {
        setQRadians(degreesToRadians(degrees));
    }

    inline void setKP(float kp) {
        int16_t kp_int = int16_t(kp/25.6*32768);
        cmd.fKP[0] = uint8_t((kp_int>>0)&0xFF);
        cmd.fKP[1] = uint8_t((kp_int>>8)&0xFF);
    }

    inline void setKD(float kd) {
        int16_t kd_int = int16_t(kd/25.6*32768);
        cmd.fKD[0] = uint8_t((kd_int>>0)&0xFF);
        cmd.fKD[1] = uint8_t((kd_int>>8)&0xFF);
    }

    inline bool isValid() {
        return fValid;
    }

    inline void setInvalid() {
        fValid = false;
    }

    bool write(int fd, const PDGoMotorCRC& motorCRC) {
        uint16_t crc = motorCRC.crc(&cmd, sizeof(cmd), fHeader[1]);
        fCRC[0] = uint8_t(crc & 0xFF);
        fCRC[1] = uint8_t(crc >> 8);
        if (::write(fd, fBytes, sizeof(fBytes)) == sizeof(fBytes)) {
            if (PDLog::isVerboseMotor()) {
                printf("[W] ");
                for (unsigned i = 0; i < sizeof(fBytes); i++) {
                    printf("%02X ", fBytes[i]);
                }
                printf("\n");
            }
            return true;
        }
        return false;
    }

    union {
        struct {
            uint8_t  fHeader[2];
            struct {
                uint8_t  fModeID;
                uint8_t  fTau[2];   // Desired joint output torque unit: Nm (q8)
                uint8_t  fDQ[2];    // Desired joint output speed unit: rad/s (q7)
                uint8_t  fQ[4];     // Desired joint output position unit: rad (q15)
                uint8_t  fKP[2];    // Expected joint stiffness coefficient unit: 0.0-1.0 (q15)
                uint8_t  fKD[2];    // Desired joint damping coefficient unit: 0.0-1.0 (q15)
            } cmd;
            uint8_t  fCRC[2];
        };
        uint8_t fBytes[17];
    };
    bool fValid = false;
};

struct PDGoMotorFeedback {

    static constexpr float GEAR_RATIO = 6.33;

    inline void init() {
        invalid();
    }

    inline uint8_t getMotorID() const {
        return (cmd.fModeID&0xF);
    }

    inline float getTau() const {
        return float(int16_t((cmd.fTau[1]<<8)|cmd.fTau[0]))/256;
    }

    inline float getDQ() const {
        return float(int16_t((cmd.fDQ[1]<<8)|cmd.fDQ[0]))*25.6/32768.0;
    }

    inline float getQ() const {
        return float(int32_t((cmd.fQ[3]<<24)|(cmd.fQ[2]<<16)|(cmd.fQ[1]<<8)|cmd.fQ[0])*6.2832/32768.0);
    }

    inline float getCurrentAngle() const {
        return radiansToDegrees(getQ() / GEAR_RATIO);
    }

    inline int getTemperature() const {
        return cmd.fTemp;
    }

    inline int getError() const {
        return cmd.fErrorFlag;
    }

    inline int getFootForce() const {
        return cmd.fForce;
    }

    inline bool isValid() const {
        return cmd.fModeID != ~0;
    }

    inline void invalid() {
        cmd.fModeID = ~0;
    }

    bool read(int fd, const PDGoMotorCRC& motorCRC) {
        if (::read(fd, fBytes, sizeof(fBytes)) == sizeof(fBytes)) {
            if (PDLog::isVerboseMotor()) {
                printf("[R] ");
                for (unsigned i = 0; i < sizeof(fBytes); i++) {
                    printf("%02X ", fBytes[i]);
                }
                printf("\n");
            }
            uint16_t crc = motorCRC.crc(&cmd, sizeof(cmd));
            if (fCRC[0] == uint8_t(crc & 0xFF) && fCRC[1] == uint8_t(crc >> 8)) {
                return true;
            } else {
                fprintf(stderr, "BAD CRC EXPECTED %02X:%02X GOT %02X:%02X\n", fCRC[0], fCRC[1], uint8_t(crc & 0xFF), uint8_t(crc >> 8));
            }
        }
        invalid();
        return false;
    }

    enum {
        kNone = 0,
        kOverHeating = 1,
        kOverCurrent = 2,
        kOverVoltage = 3,
        kEncoderFailure = 4,
        kReserved1 = 5,
        kReserved2 = 6,
        kReserved3 = 7
    };

    union {
        struct {
            struct {
                uint8_t  fHeader[2];
                uint8_t  fModeID;
                uint8_t  fTau[2];       // Actual joint output torque unit: Nm (q8)
                uint8_t  fDQ[2];        // Actual joint output speed unit: rad/s (q7)
                uint8_t  fQ[4];         // Actual joint output position unit: W (q15)
                int8_t   fTemp;         // Motor temperature: -128~127°C, temperature protection is triggered at 90°C
                uint8_t  fErrorFlag:3;  // Motor error identification
                uint16_t fForce:12;     // Foot pressure sensor data 12bit (0-4095)
                uint8_t  fReserved:1;   // reserved bits
            } cmd;
            uint8_t  fCRC[2];
        };
        uint8_t fBytes[16];
    };
};

