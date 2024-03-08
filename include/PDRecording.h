#pragma once

#include "PDLog.h"
#include <vector>

class PDRecording {
public:
    PDRecording(PDLeg& leg) :
        fLeg(leg)
    {
    }

    struct Sample {
        uint32_t    fElapsed;
        PDLeg::Pose fPose;
    };
    typedef std::vector<Sample> SampleRecording;

    bool start() {
        clear();
        fPoseSample.fElapsed = 0;
        fLeg.getPose(fPoseSample.fPose);
        fTimeStamp = currentTimeMillis();
        fRecording = true;
        return true;
    }

    bool stop() {
        if (fRecording) {
            fRecording = false;
            return true;
        }
        return false;
    }

    void clear() {
        stop();
        fSamples.clear();
        fPoseSample = Sample();
    }

    bool isRecording() const {
        return fRecording;
    }

    PDLeg* getLeg() const {
        return &fLeg;
    }

    const SampleRecording& getSamples() const {
        return fSamples;
    }

    bool update() {
        if (!fRecording)
            return false;
        PDLeg::Pose pose;
        fLeg.getPose(pose);
        if (pose != fPoseSample.fPose) {
            uint64_t now = currentTimeMillis();
            if (fSamples.size() == 0) {
                fPoseSample.fElapsed = 0;
            } else {
                fPoseSample.fElapsed = now - fTimeStamp;
            }
            fPoseSample.fPose = pose;
            fTimeStamp = now;
            fSamples.push_back(fPoseSample);
            if (PDLog::isVerbose()) {
                printf("add pose: %f, %f, %f, %f, %f\n", pose.fPositions[0], pose.fPositions[1], pose.fPositions[2], pose.fPositions[3], pose.fPositions[4]);
            }
        }
        return true;
    }

    void dump() {
        for (auto sample : fSamples) {
            PDLeg::Pose& pose = sample.fPose;
            printf("[%u]: %f, %f, %f, %f, %f\n", sample.fElapsed, pose.fPositions[0], pose.fPositions[1], pose.fPositions[2], pose.fPositions[3], pose.fPositions[4]);
        }
    }

private:
    PDLeg& fLeg;
    bool fRecording = false;
    uint64_t fTimeStamp = 0;
    Sample fPoseSample;
    SampleRecording fSamples;
};
