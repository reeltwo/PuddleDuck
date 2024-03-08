#pragma once

class PDLog {
public:
	bool parse(const char* arg) {
        if (strcmp(arg, "-v") == 0) {
            fVerbose = true;
        } else if (strcmp(arg, "-v:move") == 0) {
            fVerboseMove = true;
        } else if (strcmp(arg, "-v:motor") == 0) {
            fVerboseMotor = true;
        } else if (strcmp(arg, "-v:pos") == 0) {
            fVerbosePosition = true;
        } else {
        	return false;
        }
        return true;
	}

	static inline bool isVerbose() {
		return log().fVerbose;
	}

	static inline bool isVerboseMotor() {
		return log().fVerboseMotor;
	}

	static inline bool isVerboseMove() {
		return log().fVerboseMove;
	}

	static inline bool isVerbosePosition() {
		return log().fVerbosePosition;
	}

	static PDLog& log() {
		static PDLog sLog;
		return sLog;
	}

private:
	PDLog() {}

	bool fVerbose = false;
	bool fVerboseMove = false;
	bool fVerboseMotor = false;
	bool fVerbosePosition = false;
};
