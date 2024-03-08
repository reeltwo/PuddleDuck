#include "PDRobot.h"
#include "PDPlayback.h"
#include "PDRobot.h"

/////////////////////////////////////////////

static bool sVerbose;
static bool sVeryVerbose;

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
    fprintf(stderr, "Usage:\n%s: [-v] [-vv] [-f] [-h]\n", argv0);
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
    uint64_t relaxAfter = 0;
    bool quit = false;
    bool firstTime = true;

    PDLeg::Pose leftPose;
    PDLeg::Pose rightPose;
    PDPlayback player;
    PDRecording recording(robot.left);
    while (!quit) {
        robot.update();

        uint64_t now = currentTimeMillis();
        if (relaxAfter && relaxAfter < now) {
            robot.relax();
            relaxAfter = 0;
            if (player.stop()) {
                printf("END PLAYBACK\n");
            }
            if (recording.stop()) {
                printf("END RECORDING\n");
            }
            printf("RELAX\n");
        } else if (recording.update()) {
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
                relaxAfter = 0;
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
                relaxAfter = now + 30*1000;
                break;
            case 'v':
                sVerbose = !sVerbose;
                break;
            case -1:
                break;
            default:
                printf("RELAX\n");
                robot.relax();
                relaxAfter = 0;
                break;
        }
    }
    robot.relax();
    robot.update();
    setNonCanonicalMode(false);
    return 0;
#if 0
#if defined(__x86_64__) || defined(_M_X64)
    int pos = 0;
    if (argc >= 2) {
        pos = atoi(argv[1]);
    }

    PDSerialPort    serial("/dev/ttyUSB0");
    PDActuator      hipPitch(MOTOR_ID_HIP_PITCH, "hip.pitch");
    hipPitch.setRange(183, 29);
    if (!hipPitch.update(serial)) {
        fprintf(stderr, "Motor not responding\n");
        exit(1);
    }

    signal(SIGINT, Handler);
    setNonCanonicalMode(true);
    printf("start position:%f [degrees:%f]\n", hipPitch.getPosition(), hipPitch.getDegrees());
    hipPitch.setEasing(Easing::QuinticEaseInOut);
    hipPitch.moveToPosition(0, 4000, float(pos)/100);
    while (readKeyIfAvailable() == -1) {
        printf("%f [%f]       \r", hipPitch.getPosition(), hipPitch.getDegrees());
        hipPitch.update(serial);
    }
    printf("\n");
    hipPitch.stop(serial);
    printf("end position:%f [degrees:%f]\n", hipPitch.getPosition(), hipPitch.getDegrees());
    setNonCanonicalMode(false);
#elif defined(__x86_64__) || defined(_M_X64)
    int pos = 0;
    if (argc >= 2) {
        pos = atoi(argv[1]);
    }

    PDSerialPort    serial("/dev/ttyUSB0");
    PDActuator      knee(MOTOR_ID_KNEE, "knee");
    knee.setRange(133, 222);
    if (!knee.update(serial)) {
        fprintf(stderr, "Motor not responding\n");
        exit(1);
    }
    printf("start position:%f [degrees:%f]\n", knee.getPosition(), knee.getDegrees());
    knee.setEasing(Easing::QuinticEaseInOut);
    knee.moveToPosition(0, 4000, float(pos)/100);

    signal(SIGINT, Handler);
    setNonCanonicalMode(true);
    uint64_t startTime = millis();
    uint64_t stopTime = startTime + 2000;
    while (readKeyIfAvailable() == -1) {    
        knee.update(serial);
    }
    knee.stop(serial);
    printf("end position:%f [degrees:%f]\n", knee.getPosition(), knee.getDegrees());
    setNonCanonicalMode(false);
#elif defined(__x86_64__) || defined(_M_X64)
    int pos = 0;
    if (argc >= 2) {
        pos = atoi(argv[1]);
    }

    PDSerialPort    serial("/dev/ttyUSB0");
    PDActuator      ankle(MOTOR_ID_ANKLE_PITCH, "ankle");
    ankle.setRange(-76, -181);
    if (!ankle.update(serial)) {
        fprintf(stderr, "Motor not responding\n");
        exit(1);
    }
    printf("start position:%f [degrees:%f]\n", ankle.getPosition(), ankle.getDegrees());
    ankle.setEasing(Easing::QuinticEaseInOut);
    ankle.moveToPosition(0, 1000, float(pos)/100);

    signal(SIGINT, Handler);
    setNonCanonicalMode(true);
    uint64_t startTime = millis();
    uint64_t stopTime = startTime + 2000;
    while (readKeyIfAvailable() == -1) {    
        ankle.update(serial);
    }
    ankle.stop(serial);
    printf("end position:%f\n", ankle.getPosition());
    setNonCanonicalMode(false);
#else
    int pos = 0;
    if (argc >= 2) {
        if (strcmp(argv[1], "-v") == 0) {
            sVerbose = true;
        }
        else if (strcmp(argv[1], "-vv") == 0) {
            sVeryVerbose = true;
        }
    }

    PDLeg left("left", "/dev/ttyUSB0");
    left.fAnklePitch.setRange(-76, -181);
    left.fKnee.setRange(133, 222);
    left.fHipPitch.setRange(-6, -135);

    PDLeg right("right", "/dev/ttyUSB1");
    if (!left.update() || !right.update()) {
        fprintf(stderr, "Missing motors\n");
        return 1;
    }
    signal(SIGINT, Handler);
    setNonCanonicalMode(true);
    uint64_t relaxAfter = 0;
    bool quit = false;
    bool firstTime = true;
    LegPose leftPose;
    LegPose rightPose;
    while (!quit && left.update() && right.update()) {
        if (relaxAfter && relaxAfter < millis()) {
            printf("RELAX\n");
            left.relax();
            right.relax();
            relaxAfter = 0;
        }
        switch (readKeyIfAvailable()) {
            case 'q':
                printf("QUIT\n");
                quit = true;
                break;
            case 'a':
                printf("STAND\n");
                left.stand();
                right.stand();
                relaxAfter = 0;
                break;
            case 's':
                if (firstTime) {
                    printf("STAND FOR 30 SECONDS\n");
                    left.getPose(leftPose);
                    right.getPose(rightPose);
                    firstTime = false;
                } else {
                    printf("STAND FOR 30 SECONDS\n");
                    left.setPose(leftPose, 2000);
                    right.setPose(rightPose, 2000);
                }
                left.stand();
                right.stand();
                relaxAfter = millis() + 30*1000;
                break;
            case 'v':
                sVerbose = !sVerbose;
                break;
            case -1:
                break;
            default:
                printf("RELAX\n");
                left.relax();
                right.relax();
                relaxAfter = 0;
                break;
        }
    }
    left.relax();
    right.relax();
    left.update();
    right.update();
    setNonCanonicalMode(false);
    return 0;

    // if (pos == 1) {
    //     leftAnkle.setAbsolutePosition(30);
    //     leftKnee.setAbsolutePosition(-50);
    //     leftHip.setAbsolutePosition(-100);
    // } else {        
    //     leftAnkle.setAbsolutePosition(0);
    //     leftKnee.setAbsolutePosition(0);
    //     leftHip.setAbsolutePosition(0);
    // }
    // while (readKeyIfAvailable() == -1) {
    //     usleep(200);
    //     leftAnkle.update();
    //     leftKnee.update();
    //     leftHip.update();
    //     printf("leftKnee: %f %f %f        \r", leftAnkle.currentAngle(), leftKnee.currentAngle(), leftHip.currentAngle());
    // }
    // printf("\n");
    // leftAnkle.stop();
    // leftKnee.stop();
    // leftHip.stop();
    // setNonCanonicalMode(false);
#endif
    return 0;
#endif
}
