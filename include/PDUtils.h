#pragma once

#include "config.h"
#include <cmath>
#include <time.h>
#include <stdint.h>
#include <unistd.h>
#include <termios.h>
#include <signal.h>
#include <sys/select.h>

typedef std::string PDString;

int readKeyIfAvailable() {
    fd_set readfds;
    struct timeval timeout;
    int retval;
    char ch;

    // Clear the set ahead of time
    FD_ZERO(&readfds);

    // Add our descriptor to the set
    FD_SET(STDIN_FILENO, &readfds);

    // Set timeout to 0, which makes select non-blocking
    timeout.tv_sec = 0;
    timeout.tv_usec = 0;

    // See if there is any data available to read from stdin
    retval = select(STDIN_FILENO + 1, &readfds, NULL, NULL, &timeout);
    return (retval > 0 && read(STDIN_FILENO, &ch, 1) == 1) ? ch : -1;
}

void setNonCanonicalMode(bool enable) {
    static bool initialized;
    static struct termios oldt, newt;

    if (enable) {
        // Get the terminal settings
        tcgetattr(STDIN_FILENO, &oldt);
        // Copy settings to newt
        newt = oldt;
        // Disable canonical mode and local echo
        newt.c_lflag &= ~(ICANON | ECHO);
        // Set the new settings immediately
        tcsetattr(STDIN_FILENO, TCSANOW, &newt);
        initialized = true;
    } else if (initialized) {
        // Restore the old settings
        tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    }
}

uint64_t currentTimeMillis()
{
    uint64_t millis;
#if defined(HAVE_CLOCK_MONOTONIC)
    timespec tm;
    clock_gettime(CLOCK_MONOTONIC, &tm);
    millis = (uint64_t)tm.tv_sec * (uint64_t)1000 + (uint64_t)tm.tv_nsec / (uint64_t)1000000;
#elif defined(HAVE_CLOCK_REALTIME)
    timespec tm;
    clock_gettime(CLOCK_REALTIME, &tm);
    millis = (uint64_t)tm.tv_sec * (uint64_t)1000 + (uint64_t)tm.tv_nsec / (uint64_t)1000000;
#elif defined(HAVE_GETTIMEOFDAY)
    struct timeval tm;
    gettimeofday(&tm, 0);
    millis = (uint64_t)tm.tv_sec * (uint64_t)1000 + (uint64_t)tm.tv_usec / (uint64_t)1000;
#elif defined(HAVE_FTIME)
    struct timeb tm;
    ftime(&tm);
    millis = (uint64_t)tm.time * (uint64_t)1000 + (uint64_t)tm.millitm;
#else
    #error No timing function defined
#endif
    return millis;
}

double degreesToRadians(double degrees) {
    return degrees * (M_PI / 180.0);
}

double radiansToDegrees(double radians) {
    return radians * (180.0 / M_PI);
}

int normalize(int degrees)
{
    degrees = fmod(degrees, 360);
    if (degrees < 0)
        degrees += 360;
    return degrees;
}

int shortestDistance(int origin, int target)
{
    int result = 0.0;
    int diff = fmod(fmod(abs(origin - target), 360), 360);

    if (diff > 180)
    {
        //There is a shorter path in opposite direction
        result = (360 - diff);
        if (target > origin)
            result *= -1;
    }
    else
    {
        result = diff;
        if (origin > target)
            result *= -1;
    }
    return result;
}
