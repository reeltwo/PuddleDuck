#pragma once

#include "PDConfig.h"

#define DEFAULT_LEFT_ADAPTER    "/dev/ttyUSB0"
#define DEFAULT_RIGHT_ADAPTER   "/dev/ttyUSB1"
#define DEFAULT_LEFT_BUS        "left_bus"
#define DEFAULT_RIGHT_BUS       "right_bus"
#define DEFAULT_NECK_BUS        DEFAULT_LEFT_BUS
#define MOTOR_ID_HIP_YAW        5
#define MOTOR_ID_HIP_ROLL       4
#define MOTOR_ID_HIP_PITCH      3
#define MOTOR_ID_KNEE_PITCH     2
#define MOTOR_ID_ANKLE_PITCH    1
#define MOTOR_ID_NECK           0

// Default robot configuration
PDConfig::Robot sRobotConfig = {
    .bus = {
        {
            .name = DEFAULT_LEFT_BUS,
            .type = PDConfig::kGoMotor,
            .adapter = DEFAULT_LEFT_ADAPTER,
            .version = 1
        },
        {
            .name = DEFAULT_RIGHT_BUS,
            .type = PDConfig::kGoMotor,
            .adapter = DEFAULT_RIGHT_ADAPTER,
            .version = 1
        }
    },
    .neck = {
        .bus = DEFAULT_NECK_BUS,
        .id = MOTOR_ID_NECK,
        .range = {{NAN, NAN}},
        .kp = 1.0,
        .kd = 0.01,
        .tau = 0,
        .invert = false
    },
    .leg = {
        .left = {
            .bus = DEFAULT_LEFT_BUS,
            .ankle = {
                .pitch = {
                    .id = MOTOR_ID_ANKLE_PITCH,
                    .range = {{NAN, NAN}},
                    .kp = 1.0,
                    .kd = 0.01,
                    .tau = 0,
                    .invert = false
                }
            },
            .knee = {
                .pitch = {
                    .id = MOTOR_ID_KNEE_PITCH,
                    .range = {{NAN, NAN}},
                    .kp = 1.0,
                    .kd = 0.01,
                    .tau = 0,
                    .invert = false
                }
            },
            .hip = {
                .pitch = {
                    .id = MOTOR_ID_HIP_PITCH,
                    .range = {{NAN, NAN}},
                    .kp = 1.0,
                    .kd = 0.01,
                    .tau = 0,
                    .invert = false
                },
                .roll = {
                    .id = MOTOR_ID_HIP_ROLL,
                    .range = {{NAN, NAN}},
                    .kp = 1.0,
                    .kd = 0.01,
                    .tau = 0,
                    .invert = false
                },
                .yaw = {
                    .id = MOTOR_ID_HIP_YAW,
                    .range = {{NAN, NAN}},
                    .kp = 1.0,
                    .kd = 0.01,
                    .tau = 0,
                    .invert = false
                }
            }
        },
        .right = {
            .bus = DEFAULT_RIGHT_BUS,
            .ankle = {
                .pitch = {
                    .id = MOTOR_ID_ANKLE_PITCH,
                    .range = {{NAN, NAN}},
                    .kp = 1.0,
                    .kd = 0.01,
                    .tau = 0,
                    .invert = false
                }
            },
            .knee = {
                .pitch = {
                    .id = MOTOR_ID_KNEE_PITCH,
                    .range = {{NAN, NAN}},
                    .kp = 1.0,
                    .kd = 0.01,
                    .tau = 0,
                    .invert = false
                }
            },
            .hip = {
                .pitch = {
                    .id = MOTOR_ID_HIP_PITCH,
                    .range = {{NAN, NAN}},
                    .kp = 1.0,
                    .kd = 0.01,
                    .tau = 0,
                    .invert = false
                },
                .roll = {
                    .id = MOTOR_ID_HIP_ROLL,
                    .range = {{NAN, NAN}},
                    .kp = 1.0,
                    .kd = 0.01,
                    .tau = 0,
                    .invert = false
                },
                .yaw = {
                    .id = MOTOR_ID_HIP_YAW,
                    .range = {{NAN, NAN}},
                    .kp = 1.0,
                    .kd = 0.01,
                    .tau = 0,
                    .invert = false
                }
            }
        }
    }
};
