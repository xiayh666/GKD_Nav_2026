#pragma once
#ifndef PROTOCAL_H
#define PROTOCAL_H

#include <string>

enum ROBOT_MODE
{
    ROBOT_NO_FORCE = 0,
    ROBOT_FINISH_INIT = 1,
    ROBOT_FOLLOW_GIMBAL = 2,
    ROBOT_SEARCH = 3,
    ROBOT_IDLE = 4,
    ROBOT_NOT_FOLLOW = 5
};

struct ReceiveNavigationInfo
{
    uint8_t header;
    float yaw;
    float hp;
    bool start;
    bool auto_aim_captured;
} __attribute__((packed));

struct SendNavigationInfo
{
    uint8_t header;
    float vx;
    float vy;
    int wz;
    // bool initialised;

    ROBOT_MODE robot_mode;
    int enable_auto_aim;
} __attribute__((packed));

#endif