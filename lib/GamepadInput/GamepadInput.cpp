#include <Arduino.h>
#include <Bluepad32.h>
 
#include <GamepadInput.h>

namespace {

ControllerPtr g_controllers[BP32_MAX_GAMEPADS];

constexpr float kMaxLinearSpeedMmS = 300.0f;
constexpr float kMaxAngularSpeedRadS = 1.5f;
constexpr float kDeadzone = 0.15f;
constexpr uint32_t kPrintIntervalMs = 200;
constexpr uint32_t kControllerTimeoutMs = 250;
constexpr bool kEnableControllerWhitelist = true;
constexpr uint8_t kAllowedControllerBtAddr[6] = {0x40, 0xE4, 0x04, 0x18, 0x5F, 0x9E};

uint32_t g_lastControllerDataMs = 0;

float normalizeAxis(int raw) {
    // 对称钳位到 [-511, 511] 并除以 511,使正负向都能达到 ±1.0。
    // (摇杆原始范围约为 [-512, 511],直接除以 512 会导致正向最大仅 0.998、负向 -1.0 的不对称)
    if (raw > 511) {
        raw = 511;
    } else if (raw < -511) {
        raw = -511;
    }

    const float normalized = static_cast<float>(raw) / 511.0f;
    if (fabsf(normalized) < kDeadzone) {
        return 0.0f;
    }
    return normalized;
}

void printBtAddress(const uint8_t btaddr[6]) {
    Serial.printf("%02X:%02X:%02X:%02X:%02X:%02X",
                  btaddr[0],
                  btaddr[1],
                  btaddr[2],
                  btaddr[3],
                  btaddr[4],
                  btaddr[5]);
}

bool isAllowedController(ControllerPtr ctl) {
    if (!kEnableControllerWhitelist) {
        return true;
    }

    const ControllerProperties props = ctl->getProperties();
    for (int i = 0; i < 6; ++i) {
        if (props.btaddr[i] != kAllowedControllerBtAddr[i]) {
            return false;
        }
    }
    return true;
}

void onConnectedController(ControllerPtr ctl) {
    const ControllerProperties props = ctl->getProperties();

    Serial.print("Controller BT address: ");
    printBtAddress(props.btaddr);
    Serial.println();

    if (!isAllowedController(ctl)) {
        Serial.print("Controller rejected, not in whitelist: ");
        printBtAddress(props.btaddr);
        Serial.println();
        ctl->disconnect();
        return;
    }

    for (int i = 0; i < BP32_MAX_GAMEPADS; ++i) {
        if (g_controllers[i] == nullptr) {
            g_controllers[i] = ctl;
            g_lastControllerDataMs = millis();
            Serial.printf("Controller connected, slot: %d\n", i);
            Serial.printf("Model: %s\n", ctl->getModelName().c_str());
            return;
        }
    }

    Serial.println("Controller slots full, ignoring new device.");
}

void onDisconnectedController(ControllerPtr ctl) {
    for (int i = 0; i < BP32_MAX_GAMEPADS; ++i) {
        if (g_controllers[i] == ctl) {
            g_controllers[i] = nullptr;
            Serial.printf("Controller disconnected, slot: %d\n", i);
            return;
        }
    }
}

ControllerPtr getActiveController() {
    for (int i = 0; i < BP32_MAX_GAMEPADS; ++i) {
        if (g_controllers[i] != nullptr && g_controllers[i]->isConnected()) {
            return g_controllers[i];
        }
    }
    return nullptr;
}

void fillButtons(ControllerPtr ctl, InputState& state) {
    state.buttons.a = ctl->a();
    state.buttons.b = ctl->b();
    state.buttons.x = ctl->x();
    state.buttons.y = ctl->y();
    state.buttons.l1 = ctl->l1();
    state.buttons.r1 = ctl->r1();
    state.buttons.start = ctl->miscStart();
    state.buttons.select = ctl->miscSelect();
    state.buttons.thumbL = ctl->thumbL();
    state.buttons.thumbR = ctl->thumbR();

    const uint8_t dpad = ctl->dpad();
    state.buttons.dpadX = ((dpad & DPAD_RIGHT) ? 1 : 0) - ((dpad & DPAD_LEFT) ? 1 : 0);
    state.buttons.dpadY = ((dpad & DPAD_UP) ? 1 : 0) - ((dpad & DPAD_DOWN) ? 1 : 0);
}

void fillSticksAndTriggers(ControllerPtr ctl, InputState& state) {
    state.sticks.lx = ctl->axisX();
    state.sticks.ly = ctl->axisY();
    state.sticks.rx = ctl->axisRX();
    state.sticks.ry = ctl->axisRY();
    state.triggers.left = ctl->brake();
    state.triggers.right = ctl->throttle();
}

void fillChassisCommand(InputState& state) {
    const float leftY = -normalizeAxis(state.sticks.ly);
    const float leftX = -normalizeAxis(state.sticks.lx);
    const float rightX = -normalizeAxis(state.sticks.rx);

    state.chassis.vx_mm_s = leftY * kMaxLinearSpeedMmS;
    state.chassis.vy_mm_s = leftX * kMaxLinearSpeedMmS;
    state.chassis.wz_rad_s = rightX * kMaxAngularSpeedRadS;
}

void fillArmCommand(InputState& state) {
    // Visual servo trigger: A button held
    state.armServoTrigger = state.buttons.a;

    // Reserved arm command fields (not used in visual servo mode)
    state.arm.joint1 = 0.0f;
    state.arm.joint2 = 0.0f;
    state.arm.joint3 = 0.0f;
    state.arm.joint4 = 0.0f;
    state.arm.gripper = 0.0f;
}

void printDebug(const InputState& state) {
    static uint32_t lastPrint = 0;
    if (millis() - lastPrint < kPrintIntervalMs) {
        return;
    }

    Serial.printf(
        "cmd vx=%.1f mm/s, vy=%.1f mm/s, wz=%.2f rad/s | A:%d B:%d X:%d Y:%d LB:%d RB:%d LT:%d RT:%d\n",
        state.chassis.vx_mm_s,
        state.chassis.vy_mm_s,
        -state.chassis.wz_rad_s,
        state.buttons.a,
        state.buttons.b,
        state.buttons.x,
        state.buttons.y,
        state.buttons.l1,
        state.buttons.r1,
        state.triggers.left,
        state.triggers.right);
    lastPrint = millis();
}

}  // namespace

namespace GamepadInput {

void begin() {
    BP32.setup(&onConnectedController, &onDisconnectedController);

    // Uncomment once if you need to clear stored pairings.
    // BP32.forgetBluetoothKeys();
}

void update() {
    BP32.update();
}

InputState read() {
    InputState state;
    state.timestampMs = millis();

    ControllerPtr activeController = getActiveController();
    if (activeController == nullptr) {
        return state;
    }

    state.connected = true;
    state.hasFreshData = activeController->hasData();

    // [ ! ] 超时判定基于"链路是否在线", 不基于"有没有新数据帧"。
    //   Bluepad32 的 hasData() 在手柄状态不变时返回 false (手柄静止 = 无新包),
    //   若用它更新时间戳, 则手动瞄准 (P_AIM) 长时间不碰键会被误判断流 → 误触发急停、
    //   把放置流程打回 IDLE。改为: 只要 getActiveController() 仍返回有效 (isConnected),
    //   本帧就视为链路在线、刷新时间戳。真正断连时 getActiveController() 返回 nullptr,
    //   上面已走 connected=false 分支急停, 走不到这里。
    g_lastControllerDataMs = state.timestampMs;

    if (state.hasFreshData) {
        fillSticksAndTriggers(activeController, state);
        fillButtons(activeController, state);
        fillChassisCommand(state);
        fillArmCommand(state);
        // printDebug(state);   // 手柄命令调试刷屏, 平时关闭; 需要时取消注释
        return state;
    }

    // 无新数据帧 (手柄静止, 非断流): hasFreshData=false 供 main 区分底盘是否更新;
    // 链路仍在线, 不判超时, 故放置流程/手动瞄准可长时间保持不被打断。
    return state;
}

}  // namespace GamepadInput
