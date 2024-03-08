#pragma once

#include <cmath>
#include "PDLog.h"
#include "PDEasing.h"
#include "PDGoMotorBus.h"
#include "PDUtils.h"

class PDGoActuator {
public:
    PDGoActuator() {
        fName[0] = '\0';
    }

    PDGoActuator(uint8_t id, const char* name = nullptr) {
        setMotorID(id, name);
    }

    void setBus(PDGoMotorBus* bus) {
        fBus = bus;
    }

    inline void setMotorID(uint8_t id, const char* name = nullptr) {
        fMotorID = id;
        fName[0] = '\0';
        if (name != nullptr) {
            snprintf(fName, sizeof(fName), "%s", name);
        }
    }

    double getMinimum() const
    {
        return std::min(fRange[0], fRange[1]);
    }

    double getMaximum() const
    {
        return std::max(fRange[0], fRange[1]);
    }

    void setRange(double pos1, double pos2) {
        fRange[0] = pos1;
        fRange[1] = pos2;
    }

    void setKP(double kp) {
        fKP = kp;
    }

    void setKD(double kd) {
        fKD = kd;
    }

    void setTau(double tau) {
        fTau = tau;
    }

    inline bool isRangeValid() const {
        return (fRange[0] != fRange[1]);
    }

    double scaleToPos(double scale) {
        scale = std::min(std::max(0.0, scale), 1.0);
        if (fRange[0] < fRange[1]) {
            return fRange[0] + (fRange[1] - fRange[0]) * scale;
        }
        return fRange[0] - (fRange[0] - fRange[1]) * scale;
    }

    void moveToPosition(uint32_t startDelay, uint32_t moveTime, double scale) {
        moveToDegrees(startDelay, moveTime, scaleToPos(scale));
    }

    void moveToDegrees(uint32_t startDelay, uint32_t moveTime, double degrees)
    {
        if (!isRangeValid()) {
            fprintf(stderr, "Actuator range has not been set\n");
            return;
        }
        fActive = true;
        double finalPos = std::min(getMaximum(), std::max(getMinimum(), degrees));
        if (moveTime != 0) {
            uint64_t timeNow = currentTimeMillis();
            fStartTime = startDelay + timeNow;
            fFinishTime = moveTime + fStartTime;
            fOffTime = fFinishTime;
            if (moveTime == 0)
                fOffTime += 200;
            fFinishPos = finalPos;
            fPosNow = fDegrees;
            fStartPosition = fPosNow;
            fDeltaPos = fFinishPos - fPosNow;
            fLastMoveTime = currentTimeMillis();
        } else {
            fPosNow = finalPos;
        }
    }

    void move(uint64_t timeNow)
    {
        Easing::Method easing = Easing::get(fEasingMethod);
        if (fFinishTime != 0)
        {
            if (timeNow < fStartTime)
            {
                /* wait */
            }
            else if (timeNow >= fFinishTime)
            {
                fPosNow = fFinishPos;
                reset();
            }
            else if (fLastMoveTime != timeNow)
            {
                uint32_t timeSinceLastMove = timeNow - fStartTime;
                uint32_t denominator = fFinishTime - fStartTime;
                double fractionChange = easing(double(timeSinceLastMove) / double(denominator));
                double distanceToMove = fDeltaPos * fractionChange;
                double newPos = fStartPosition + distanceToMove;
                if (newPos != fPosNow)
                {
                    fPosNow = fStartPosition + distanceToMove;
                    fLastMoveTime = timeNow;
                }
            }
        }
        else if (fOffTime != 0 && timeNow >= fOffTime)
        {
            fOffTime = 0;
        }
    }

    bool isMoving() const
    {
        if (fActive)
            return (fFinishTime != 0 || fLastMoveTime != 0 || fFinishPos != 0);
        return false;
    }

    void reset()
    {
        fFinishTime = 0;
        fLastMoveTime = 0;
        fFinishPos = 0;
        fOffTime = 0;
    }

    void update(PDGoMotorCmd& cmd, PDGoMotorFeedback& feedback) {
        if (fIgnore) {
            cmd.setInvalid();
            return;
        }
        move(currentTimeMillis());
        cmd.setMotorID(fMotorID);
        if (fActive) {
            if (PDLog::isVerboseMove())
                printf("[%s]: %f\n", getName(), fPosNow);
            cmd.setFOCMode();
            cmd.setKP(fKP);
            cmd.setKD(fKD);
            cmd.setQRadians(degreesToRadians(fPosNow));
            cmd.setTau(fTau);
        } else {
            cmd.setBrakeMode();
            cmd.setKP(0.00);
            cmd.setKD(0.00);
            cmd.setQRadians(0);
            cmd.setTau(0.0);
        }
        cmd.setDQ(0);
        feedback.init();
    }

    bool checkRange() const {
        if (std::isnan(fMinDegrees) || std::isnan(fMaxDegrees)) {
            return false;
        }
        return true;
    }

    void update(PDGoMotorFeedback& feedback) {
        if (feedback.getError() != 0) {
            fErrorCount++;
        } else if (feedback.getMotorID() != fMotorID) {
            fprintf(stderr, "WRONG MOTOR GOT %d EXPECTING %d\n", feedback.getMotorID(), fMotorID);
        } else {
            fDegrees = feedback.getCurrentAngle();
            if (!fActive) {
                fPosNow = fStartPosition = fDegrees;
            }
            if (std::isnan(fMinDegrees) || fMinDegrees < fDegrees) {
                fMinDegrees = fDegrees;
            }
            if (std::isnan(fMaxDegrees) || fMaxDegrees > fDegrees) {
                fMaxDegrees = fDegrees;
            }
            fLastResponse = currentTimeMillis();
        }
    }

    bool update() {
        if (fBus == nullptr) {
            fprintf(stderr, "UNRESOLVED ACTUATOR BUS\n");
            return false;
        }
        PDGoMotorCmd cmd;
        PDGoMotorFeedback feedback;
        update(cmd, feedback);
        if (!fBus->sendRecv(&cmd, &feedback)) {
            printf("ERROR");
            fMissCount++;
            return false;
        }
        update(feedback);
        return true;
    }

    unsigned getMissCount() const {
        return fMissCount;
    }

    unsigned getErrorCount() const {
        return fErrorCount;
    }

    bool hasError() const {
        return (fMissCount != 0 || fErrorCount != 0);
    }

    bool stop() {
        reset();
        fActive = false;
        return update();
    }

    void relax() {
        fActive = false;
    }

    void stiff(double jointStiffness = 1.0) {
        reset();
        fActive = true;
    }

    inline double getDegrees() const {
        return fDegrees;
    }

    inline double getPosition() const {
        if (isRangeValid()) {
            double position;
            if (fRange[0] < fRange[1]) {
                position = (fDegrees - fRange[0]) / (fRange[1] - fRange[0]);
            } else {
                position = -((fDegrees - fRange[0]) / (fRange[0] - fRange[1]));
            }
            if (position < -0.5 || position > 1.5) {
                fprintf(stderr, "ERROR ACUTATOR: \"%s\" OUT OF ALIGNMENT\n", fName);
                fprintf(stderr, "[%s] pos=%f degrees=%f [%f:%f]\n", fName, position, fDegrees, fRange[0], fRange[1]);
                return NAN;
            } else if (position < 0) {
                return 0;
            } else if (position > 1) {
                return 1;
            }
            return position;
        }

        return NAN;
    }

    void setEasing(Easing::Method method) {
        fEasingMethod = method;
    }

    double getMinDegrees() const {
        return fMinDegrees;
    }

    double getMaxDegrees() const {
        return fMaxDegrees;
    }

    uint8_t getID() const {
        return fMotorID;
    }

    const char* getName() const {
        return fName;
    }

    void setIgnore() {
        fIgnore = true;
    }

    inline uint32_t timeSinceLastResponse() const {
        if (fLastResponse) {
            return currentTimeMillis() - fLastResponse;
        }
        return ~0;
    }

    inline bool isResponding() const {
        return (timeSinceLastResponse() < 100);
    }

private:
    char            fName[16];
    PDGoMotorBus*   fBus = nullptr;
    unsigned        fErrorCount = 0;
    unsigned        fMissCount = 0;
    bool            fIgnore = false;
    uint8_t         fMotorID = 0;
    bool            fActive = false;
    double          fRange[2] = { 0, 0 };
    double          fMinDegrees = NAN;
    double          fMaxDegrees = NAN;
    uint64_t        fStartTime = 0;
    uint64_t        fFinishTime = 0;
    uint64_t        fLastMoveTime = 0;
    uint64_t        fOffTime = 0;
    double          fKP = 1.0;
    double          fKD = 0.01;
    double          fTau = 0;
    double          fFinishPos = 0;
    double          fStartPosition = 0;
    double          fPosNow = 0;
    double          fDeltaPos = 0;
    double          fDegrees = 0;
    uint64_t        fLastResponse = 0;
    double          (*fEasingMethod)(double completion) = nullptr;
};
