#include <stdio.h>
#include "PDGoMotorBus.h"

static int getVersion(const char* argv0) {
    return (argv0[strlen(argv0)-1] == '2') ? 2 : 1;
}

static void usage(const char* argv0) {
    printf("Go Motor ID change tool.\n");
    printf("Version: %d\n\n", getVersion(argv0));
    printf("Motor broadcast ID:15\n");
    printf("Notice: There cannot be motors with the same ID on a RS-485 bus!!!\n\n");
    printf("usage: %s [tty device] [id] [target_id]\n", argv0);
    printf("ex:    %s /dev/ttyUSB0 0 1  :Set motor 0 to id 1 [id] [target_id]\n", argv0);
    printf("ex:    %s /dev/ttyUSB0 15 1 :Set All motor to id 1 (Use with caution)\n", argv0);
}

int main(int argc, const char* argv[])
{
    if (argc < 4) {
        usage(argv[0]);
        return 0;
    }
    int version = getVersion(argv[0]);
    const char* serialPort = argv[1];
    int oldid = atoi(argv[2]);
    int newid = atoi(argv[3]);
    uint8_t buffer[] = {
        0xFB,
        1,
        uint8_t(((newid&0xF)<<4)|(oldid&0xF)),
        0xBB
    };

    if (!PDLog::log().parse("-v:motor")) {
        printf("WTF\n");
    }

    PDGoMotorBus bus(argv[1], argv[1], version);
    bus.setReadTimeout(20, sizeof(buffer));

    PDGoMotorCmd cmd;
    cmd.setBootMode();
    if (!bus.send(&cmd)) {
        return 1;
    }

    if (bus.write(buffer, sizeof(buffer)) != sizeof(buffer)) {
        fprintf(stderr, "Failed to send changeid command\n");
        return 1;
    }
    if (bus.read(buffer, sizeof(buffer)) != sizeof(buffer)) {
        fprintf(stderr, "[ERROR] Modify id %d to %d failure, No motor be found.\n", oldid, newid);
        return 1;
    }
    printf("SUCCESS\n");
    return 0;
}