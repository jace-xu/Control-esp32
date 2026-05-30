#include <Arduino.h>
#include <BottomControl.h>
#include <GamepadInput.h>

namespace {

constexpr int kMotorBaudRate = 115200;
constexpr int kMotorRxPin = 16;
constexpr int kMotorTxPin = 17;
constexpr uint32_t kFreshDataGraceMs = 60;

ControlSerial* g_control_serial = nullptr;
BottomControl* g_bottom_control = nullptr;
bool g_is_stopped = true;
bool g_timeout_reported = false;
uint32_t g_last_fresh_input_ms = 0;

void stopChassis() {
    if (g_bottom_control != nullptr) {
        g_bottom_control->stop();
    }
    g_is_stopped = true;
}

// 将手柄解析出的底盘速度命令下发给底盘控制库。
void applyChassisCommand(const InputState& input) {
    g_bottom_control->motors_control({
        input.chassis.vx_mm_s,
        input.chassis.vy_mm_s,
        input.chassis.wz_rad_s,
    });
    g_is_stopped = false;
}

// 机械臂控制预留入口：
// 后续如果机械臂也由手柄控制，可以在这里读取 input.arm / input.buttons / input.triggers
// 再调用机械臂电控对象的方法。当前先保留空实现，不影响底盘联调。
void applyArmCommand(const InputState& input) {
    (void)input;
    // Reserve this hook for future arm control integration.
}

}  // namespace

void setup() {
    Serial.begin(115200);
    delay(200);
    Serial.println("Chassis bring-up program started");

    // 初始化底盘电机控制串口。这里的 16/17 是 ESP32 的 Serial2 引脚定义：
    // GPIO16 = RX, GPIO17 = TX。
    g_control_serial = new ControlSerial(kMotorBaudRate, kMotorRxPin, kMotorTxPin);
    g_bottom_control = new BottomControl(*g_control_serial);

    // 初始化 Bluepad32 手柄输入层。
    GamepadInput::begin();

    // 上电先停车，避免上电瞬间误动作。
    stopChassis();
}

void loop() {
    // Bluepad32 必须持续 update，手柄状态才会刷新。
    GamepadInput::update();

    // 读取当前这一帧已经封装好的输入状态。
    const InputState input = GamepadInput::read();

    // 如果没有可用手柄，底盘保持停车。
    if (!input.connected) {
        if (!g_is_stopped) {
            stopChassis();
            Serial.println("No active controller, chassis stopped");
        }
        g_timeout_reported = false;
        delay(20);
        return;
    }

    // 如果手柄已连接，但超过设定时间没有新数据，也自动停车。
    if (input.timedOut) {
        if (!g_is_stopped) {
            stopChassis();
        }
        if (!g_timeout_reported) {
            Serial.println("Controller data timeout, chassis stopped");
            g_timeout_reported = true;
        }
        delay(20);
        return;
    }

    g_timeout_reported = false;

    // 只有收到新一帧手柄数据时，才更新底盘/机械臂控制输出。
    // 如果当前帧没有新数据，则立即清零底盘速度，避免沿用上一帧命令。
    if (input.hasFreshData) {
        g_last_fresh_input_ms = input.timestampMs;
        applyChassisCommand(input);
        applyArmCommand(input);
    } else if (!g_is_stopped &&
               (input.timestampMs - g_last_fresh_input_ms) > kFreshDataGraceMs) {
        stopChassis();
    }

    delay(20);
}
