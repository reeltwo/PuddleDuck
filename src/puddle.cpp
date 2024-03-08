#include "PDRobot.h"
#include "PDPlayback.h"
#include "PDRobot.h"

/////////////////////////////////////////////

static PDRobot* sActiveRobot;
static void Handler(int signo)
{
    //System Exit
    printf("\r\nHandler:Program stop\r\n"); 
    setNonCanonicalMode(false);

    if (sActiveRobot != nullptr) {
        sActiveRobot->relax();
    }
    exit(0);
}

bool saveConfiguration() {
#ifdef USE_YAML
    return sRobotConfig.save(PDCONFIG_FILE);
#endif
    fprintf(stderr, "NO PERSISTENT STORAGE FOR MOTOR CONFIGURATION\n");
    return true;
}

bool loadConfiguration() {
#ifdef USE_YAML
    if (sRobotConfig.exists(PDCONFIG_FILE)) {
        return sRobotConfig.load(PDCONFIG_FILE);
    } else {
        fprintf(stderr, "No configuration file. Loading defaults.\n");
        if (!sRobotConfig.load(PDDEFAULT_FILE)) {
            return false;
        }
        return saveConfiguration();
    }
#endif
    return true;
}

static void usage(const char* argv0) {
    fprintf(stderr, "Usage:\n%s: [-v] [-v:pos] [-v:move] [-v:motor] [-f] [-h]\n", argv0);
}

int main(int argc, const char* argv[]) {
    int pos = 0;
    bool forceContinue = false;
    for (int argi = 1; argi < argc; argi++) {
        if (strncmp(argv[argi], "-v", 2) == 0 && PDLog::log().parse(argv[argi])) {
            /* Do nothing */
        } else if (strcmp(argv[argi], "-f") == 0) {
            forceContinue = true;
        } else if (strcmp(argv[argi], "-h") == 0) {
            usage(argv[0]);
            return 0;
        } else {
            fprintf(stderr, "Unknown argument: %s\n", argv[argi]);
            usage(argv[0]);
            return 1;
        }
    }
    if (!loadConfiguration()) {
        return 1;
    }

    PDRobot robot(sRobotConfig);
    if (robot.init(forceContinue)) {
        fprintf(stderr, "FORCE CONTINUE EVEN THOUGH MOTORS ARE MISSING\n");
    } else {
        return 1;
    }
    robot.checkRanges();

    sActiveRobot = &robot;
    signal(SIGINT, Handler);
    setNonCanonicalMode(true);
    bool quit = false;
    bool firstTime = true;

    PDLeg::Pose leftPose;
    PDLeg::Pose rightPose;
    PDPlayback player;
    PDRecording recording(robot.left);
    while (!quit) {
        robot.update();

        uint64_t now = currentTimeMillis();
        if (recording.update()) {
            /* recording motion */
        } else if (player.update()) {
            /* playback */
        }
        switch (readKeyIfAvailable()) {
            case 'q':
                printf("QUIT\n");
                quit = true;
                break;
            case 'a':
                printf("STAND\n");
                robot.stand();
                break;
            case 'c':
                robot.updateJointRange(sRobotConfig);
                saveConfiguration();
                loadConfiguration();
                break;
            case 'p':
                recording.dump();
                if (recording.stop()) {
                    printf("STOPPED RECORDING\n");
                }
                player.loadSamples(recording);
                if (!player.start()) {
                    printf("NO RECORDING\n");
                }
                break;
            case 'r':
                if (player.isPlaying()) {
                    player.stop();
                    printf("STOPPED PLAYBACK\n");
                }
                if (recording.start()) {
                    printf("RECORDING\n");
                }
                break;
            case 'z':
                printf("MOVE ANKLE\n");
                robot.left.fAnklePitch.moveToPosition(0, 4000, 1.0);
                break;
            case 'x':
                printf("MOVE ANKLE\n");
                robot.left.fAnklePitch.moveToPosition(0, 4000, 0);
                break;
            case 's':
                if (firstTime) {
                    printf("STAND FOR 30 SECONDS\n");
                    robot.left.getPose(leftPose);
                    robot.right.getPose(rightPose);
                    robot.left.stand();
                    robot.right.stand();
                    firstTime = false;
                } else {
                    printf("STAND FOR 30 SECONDS\n");
                    robot.left.setPose(leftPose, 2000);
                    robot.right.setPose(rightPose, 2000);
                }
                break;
            case -1:
                break;
            default:
                printf("RELAX\n");
                robot.relax();
                player.stop();
                recording.stop();
                break;
        }
    }
    robot.relax();
    robot.update();
    setNonCanonicalMode(false);
    return 0;
}
