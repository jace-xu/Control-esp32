#pragma once

#include <stdint.h>

struct ChassisCommand {
    float vx_mm_s = 0.0f;
    float vy_mm_s = 0.0f;
    float wz_rad_s = 0.0f;
};

struct StickState {
    int lx = 0;
    int ly = 0;
    int rx = 0;
    int ry = 0;
};

struct TriggerState {
    int left = 0;
    int right = 0;
};

struct ButtonState {
    bool a = false;
    bool b = false;
    bool x = false;
    bool y = false;
    bool l1 = false;
    bool r1 = false;
    bool start = false;
    bool select = false;
    bool thumbL = false;
    bool thumbR = false;
    int dpadX = 0;
    int dpadY = 0;
};

struct ArmCommand {
    float joint1 = 0.0f;
    float joint2 = 0.0f;
    float joint3 = 0.0f;
    float joint4 = 0.0f;
    float gripper = 0.0f;
};

struct InputState {
    bool connected = false;
    bool hasFreshData = false;
    bool timedOut = false;
    uint32_t timestampMs = 0;
    StickState sticks;
    TriggerState triggers;
    ButtonState buttons;
    ChassisCommand chassis;
    ArmCommand arm;
};
