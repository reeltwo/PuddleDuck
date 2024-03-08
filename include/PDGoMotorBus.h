#pragma once

#include <string>
#include <assert.h>
#include <fcntl.h>
#include <string.h>
#include "PDUtils.h"
#include "PDGoMotorCmd.h"

class PDGoMotorBus {
public:
    PDGoMotorBus(PDString name, PDString port, int version = 1) :
        PDGoMotorBus(name.c_str(), port.c_str(), version)
    {
    }

    PDGoMotorBus(const char* name, const char* port, int version = 1) {
        snprintf(fPort, sizeof(fPort), "%s", port);
        snprintf(fName, sizeof(fName), "%s", name);
        fMotorCRC.setVersion(version);
        fd = open(port, O_RDWR | O_NOCTTY);
        if (fd <= 0) {
            fprintf(stderr, "Error opening serial port %s: %s\n", port, strerror(errno));
            return;
        }
        tcflush(fd, TCIFLUSH);

        struct termios tty;
        if(tcgetattr(fd, &tty) != 0) {
            fprintf(stderr, "Error %i from tcgetattr: %s\n", errno, strerror(errno));
            close(fd);
            fd = -1;
            return;
        }
        if (cfsetispeed(&tty, B4000000) != 0) {
            perror("cfsetispeed fd");
        }
        if (cfsetospeed(&tty, B4000000) != 0) {
            perror("cfsetospeed fd");
        }

        tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

        tty.c_cflag &= ~CSIZE; // Clear all bits that set the data size 
        tty.c_cflag |= CS8; // 8 bits per byte (most common)

        tty.c_cflag &= ~PARENB; // Clear parity bit
        tty.c_iflag &= ~INPCK;  //enable parity checking

        tty.c_cflag &= ~CSTOPB; // Clear stop field

        tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)

        tty.c_lflag &= ~ICANON;
        tty.c_lflag &= ~ECHO; // Disable echo
        tty.c_lflag &= ~ECHOE; // Disable erasure
        tty.c_lflag &= ~ECHONL; // Disable new-line echo
        tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
        tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

        tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
        tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed

        tty.c_cc[VTIME] = 2;   // Dont wait return immediately
        tty.c_cc[VMIN] = 0;

        if (tcsetattr(fd, TCSANOW, &tty) != 0) {
            perror("tcsetattr fd");
        }
        tcflush(fd, TCIFLUSH);
    }

    ~PDGoMotorBus() {
        if (fd != -1) {
            close(fd);
            fd = -1;
        }
    }

    void setReadTimeout(uint32_t centiseconds, int minAvailable = 0) {
        struct termios tty;
        if(tcgetattr(fd, &tty) != 0) {
            fprintf(stderr, "Error %i from tcgetattr: %s\n", errno, strerror(errno));
            close(fd);
            fd = -1;
            return;
        }
        tty.c_cc[VTIME] = centiseconds;
        tty.c_cc[VMIN] = minAvailable;
    }

    bool send(PDGoMotorCmd* cmd) {
        if (fd == -1) {
            return false;
        }
        if (!cmd->write(fd, fMotorCRC)) {
            fprintf(stderr, "FAILED TO WRITE MOTOR COMMAND TO %s\n", fPort);
            return false;
        }
        return true;
    }

    bool sendRecv(PDGoMotorCmd* cmd, PDGoMotorFeedback* feedback) {
        feedback->init();
        if (!cmd->isValid()) {
            return true;
        }
        if (fd == -1) {
            return false;
        }
        if (!cmd->write(fd, fMotorCRC)) {
            fprintf(stderr, "FAILED TO WRITE MOTOR COMMAND TO %s\n", fPort);
            return false;
        }
        if (!feedback->read(fd, fMotorCRC)) {
            return false;
        }
        return true;
    }

    unsigned sendRecv(unsigned count, PDGoMotorCmd* cmd, PDGoMotorFeedback* feedback) {
        unsigned successCount = 0;
        for (unsigned i = 0; i < count; i++) {
            if (sendRecv(&cmd[i], &feedback[successCount])) {
                successCount++;
            }
        }
        return successCount;
    }

    ssize_t read(void* buffer, size_t bufferSize) {
        return ::read(fd, buffer, bufferSize);
    }

    ssize_t write(const void* buffer, size_t bufferSize) {
        return ::write(fd, buffer, bufferSize);
    }

    const char* getName() {
        return fName;
    }

    int getFD() const {
        return fd;
    }

private:
    char fName[16];
    char fPort[16];
    int fd = -1;
    PDGoMotorCRC fMotorCRC;
};
