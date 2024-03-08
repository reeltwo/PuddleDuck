#pragma once

#include "PDRecording.h"

class PDPlayback {
public:
    PDPlayback() {}

    void loadSamples(const PDRecording& recording) {
        fLeg = recording.getLeg();
        fSamples = recording.getSamples();
    }

    bool start() {
        fIndex = 0;
        fNextTime = 0;
        fPlaying = false;
        if (fLeg != nullptr && fSamples.size() != 0) {
            uint64_t now = currentTimeMillis();
            fLeg->setPose(fSamples[0].fPose, 2000);
            fNextTime = now + 2000;
            fPlaying = true;
            return true;
        }
        return false;
    }

    bool stop() {
        if (fPlaying) {
            fNextTime = 0;
            fIndex = 0;
            fPlaying = false;
            if (fLeg != nullptr)
                fLeg->relax();
            return true;
        }
        return false;
    }

    bool update() {
        if (!fPlaying || fLeg == nullptr)
            return false;
        uint64_t now = currentTimeMillis();
        if (fNextTime < now) {
            if (fIndex < fSamples.size()) {
                PDLeg::Pose pose = fSamples[fIndex].fPose;
                if (PDLog::isVerbose()) {
                    printf("play pose: %f, %f, %f, %f, %f\n", pose.fPositions[0], pose.fPositions[1], pose.fPositions[2], pose.fPositions[3], pose.fPositions[4]);
                }
                fLeg->setPose(pose, 0);
                if (fIndex + 1 < fSamples.size()) {
                    fNextTime = now + fSamples[fIndex + 1].fElapsed;
                }
                fIndex++;
            } else {
                fNextTime = 0;
                fIndex = 0;
                fPlaying = false;
                fLeg->relax();
            }
        }
        return true;
    }

    bool isPlaying() const {
        return fPlaying;
    }

private:
    PDLeg* fLeg;
    std::vector<PDRecording::Sample> fSamples;
    uint64_t fNextTime = 0;
    unsigned fIndex = 0;
    bool fPlaying = false;
};
