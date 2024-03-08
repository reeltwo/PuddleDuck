#pragma once

#include "PDDefaults.h"
#include "PDGoMotorBus.h"
#include "PDGoActuator.h"

struct PDLeg {
    struct Pose {
        static constexpr double CLOSE_ENOUGH = 1e-4;
        double  fPositions[5] = { NAN, NAN, NAN, NAN, NAN };

        static constexpr size_t kNumActuators = sizeof(((Pose*)1)->fPositions)/sizeof(((Pose*)1)->fPositions[0]);

        static bool almostEqual(double a, double b, double tolerance = CLOSE_ENOUGH) {
            return std::abs(a - b) <= tolerance;
        }

        friend bool almostEqual(const Pose& lhs, const Pose& rhs, double tolerance = CLOSE_ENOUGH) {
            for (int i = 0; i < sizeof(fPositions)/sizeof(fPositions[0]); i++) {
                if (std::isnan(lhs.fPositions[i]) && std::isnan(rhs.fPositions[i]))
                    continue;
                if (!almostEqual(lhs.fPositions[i], rhs.fPositions[i], tolerance))
                    return false;
            }
            return true;
        }

        friend bool operator==(const Pose& lhs, const Pose& rhs) {
            for (int i = 0; i < sizeof(fPositions)/sizeof(fPositions[0]); i++) {
                if (std::isnan(lhs.fPositions[i]) && std::isnan(rhs.fPositions[i]))
                    continue;
                // if (lhs.fPositions[i] != rhs.fPositions[i])
                //     return false;
                if (!almostEqual(lhs.fPositions[i], rhs.fPositions[i], CLOSE_ENOUGH))
                    return false;
            }
            return true;
        }

        friend bool operator!=(const Pose& lhs, const Pose& rhs) {
            return !(lhs == rhs);
        }
    };

    PDLeg(const char* name, const PDConfig::Leg& leg) :
        fHipYaw(fActuator[4]),
        fHipRoll(fActuator[3]),
        fHipPitch(fActuator[2]),
        fKneePitch(fActuator[1]),
        fAnklePitch(fActuator[0])
    {
        snprintf(fLeg, sizeof(fLeg), "%s", name);
        fHipYaw.setMotorID(leg.hip.yaw.id, "hip.yaw");
        fHipYaw.setRange(leg.hip.yaw.range.value[0], leg.hip.yaw.range.value[1]);
        fHipYaw.setKP(leg.hip.yaw.kp);
        fHipYaw.setKD(leg.hip.yaw.kd);
        fHipYaw.setTau(leg.hip.yaw.tau);

        fHipRoll.setMotorID(leg.hip.roll.id, "hip.roll");
        fHipRoll.setRange(leg.hip.roll.range.value[0], leg.hip.roll.range.value[1]);
        fHipRoll.setKP(leg.hip.roll.kp);
        fHipRoll.setKD(leg.hip.roll.kd);
        fHipRoll.setTau(leg.hip.roll.tau);

        fHipPitch.setMotorID(leg.hip.pitch.id, "hip.pitch");
        fHipPitch.setRange(leg.hip.pitch.range.value[0], leg.hip.pitch.range.value[1]);
        fHipPitch.setKP(leg.hip.pitch.kp);
        fHipPitch.setKD(leg.hip.pitch.kd);
        fHipPitch.setTau(leg.hip.pitch.tau);

        fKneePitch.setMotorID(leg.knee.pitch.id, "knee.pitch");
        fKneePitch.setRange(leg.knee.pitch.range.value[0], leg.knee.pitch.range.value[1]);
        fKneePitch.setKP(leg.knee.pitch.kp);
        fKneePitch.setKD(leg.knee.pitch.kd);
        fKneePitch.setTau(leg.knee.pitch.tau);

        fAnklePitch.setMotorID(leg.ankle.pitch.id, "ankle.pitch");
        fAnklePitch.setRange(leg.ankle.pitch.range.value[0], leg.ankle.pitch.range.value[1]);
        fAnklePitch.setKP(leg.ankle.pitch.kp);
        fAnklePitch.setKD(leg.ankle.pitch.kd);
        fAnklePitch.setTau(leg.ankle.pitch.tau);
    }

    PDLeg(const char* name, const char* bus) :
        fHipYaw(fActuator[4]),
        fHipRoll(fActuator[3]),
        fHipPitch(fActuator[2]),
        fKneePitch(fActuator[1]),
        fAnklePitch(fActuator[0])
    {
        snprintf(fLeg, sizeof(fLeg), "%s", name);
        fHipYaw.setMotorID(MOTOR_ID_HIP_YAW, "hip.yaw");
        fHipRoll.setMotorID(MOTOR_ID_HIP_ROLL, "hip.roll");
        fHipPitch.setMotorID(MOTOR_ID_HIP_PITCH, "hip.pitch");
        fKneePitch.setMotorID(MOTOR_ID_KNEE_PITCH, "knee.pitch");
        fAnklePitch.setMotorID(MOTOR_ID_ANKLE_PITCH, "ankle.pitch");
    }

    constexpr unsigned numberOfActuators() const {
        return sizeof(fActuator)/sizeof(fActuator[0]);
    }

    const char* getName() const {
        return fLeg;
    }

    void setBus(PDGoMotorBus* bus) {
        fBus = bus;
    }

    char                fLeg[16];
    PDGoMotorBus*       fBus = nullptr;
    PDGoActuator        fActuator[5];
    PDGoMotorCmd        fMotorCommand[sizeof(fActuator)/sizeof(fActuator[0])];
    PDGoMotorFeedback   fMotorFeedback[sizeof(fActuator)/sizeof(fActuator[0])];

    PDGoActuator&       fHipYaw;
    PDGoActuator&       fHipRoll;
    PDGoActuator&       fHipPitch;
    PDGoActuator&       fKneePitch;
    PDGoActuator&       fAnklePitch;
    bool                fHasHip;

    static constexpr size_t kNumActuators = sizeof(((PDLeg*)1)->fActuator)/sizeof(((PDLeg*)1)->fActuator[0]);

    void report() {
        bool needBrackets = true;
        for (int i = 0; i < kNumActuators; i++) {
            auto pos = fActuator[i].getPosition();
            if (!std::isnan(pos)) {
                if (needBrackets)
                    printf("[");
                printf("%s%s.%s:%f",
                    (needBrackets ? "" : " "),
                    fLeg, fActuator[i].getName(), pos);
                needBrackets = false;
            }
        }
        if (!needBrackets)
            printf("]\n");
    }

    void getPose(Pose& pose) {
        for (unsigned i = 0; i < numberOfActuators(); i++) {
            pose.fPositions[i] = fActuator[i].getPosition();
        }
    }

    void setPose(Pose& pose, uint32_t moveTime) {
        for (unsigned i = 0; i < numberOfActuators(); i++) {
            if (!std::isnan(pose.fPositions[i])) {
                fActuator[i].moveToPosition(0, moveTime, pose.fPositions[i]);
            }
        }
    }

    bool update() {
        if (fBus == nullptr) {
            fprintf(stderr, "[%s] UNRESOLVED LEG BUS\n", fLeg);
            return false;
        }
        unsigned numActuators = numberOfActuators();
        for (unsigned i = 0; i < numActuators; i++) {
            fActuator[i].update(fMotorCommand[i], fMotorFeedback[i]);
        }
        uint64_t stopTime = currentTimeMillis();
        unsigned numSent = fBus->sendRecv(numActuators, fMotorCommand, fMotorFeedback);
        bool success = (numActuators == numSent);
        for (unsigned fi = 0; fi < numSent; fi++) {
            PDGoMotorFeedback* feedback = &fMotorFeedback[fi];
            if (feedback->isValid()) {
                for (unsigned mi = 0; mi < numActuators; mi++) {
                    if (feedback->getMotorID() == fActuator[mi].getID()) {
                        fActuator[mi].update(*feedback);
                        break;
                    }
                }
            } else {
                success = false;
            }
        }
        if (PDLog::isVerbosePosition()) {
            report();
        }
        return success;
    }

    void relax() {
        fHipYaw.relax();
        fHipRoll.relax();
        fHipPitch.relax();
        fKneePitch.relax();
        fAnklePitch.relax();
    }

    void stand() {
        fHipYaw.stiff();
        fHipRoll.stiff();
        fHipPitch.stiff();
        fKneePitch.stiff();
        fAnklePitch.stiff();
    }

    void setKnee(double pos, uint32_t moveTime = 2000) {
        fKneePitch.moveToDegrees(0, moveTime, pos);
        // fKneePitch.stiff();
        //fKneePitch.setAbsolutePosition(pos);
    }
    // void setAnklePosition() {
    //     left
    // }
};
