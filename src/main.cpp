#include <Arduino.h>
#include <BottomControl.h>
#include <ArmControl.h>
#include <VisionSerial.h>
#include <GamepadInput.h>

namespace {

// ============================================================================
// 硬件引脚和通信配置
// ============================================================================

constexpr int kMotorBaudRate = 115200;   // 电机控制总线波特率 (Serial2)
constexpr int kMotorRxPin = 16;          // 电机总线 RX (GPIO16)
constexpr int kMotorTxPin = 17;          // 电机总线 TX (GPIO17)
constexpr int kVisionBaudRate = 115200;  // 视觉串口波特率 (Serial1)
constexpr int kVisionRxPin = 18;         // 视觉串口 RX (GPIO18, 接树莓派 TX)
constexpr int kVisionTxPin = 19;         // 视觉串口 TX (GPIO19, 接树莓派 RX)

// ============================================================================
// 安全参数
// ============================================================================

// 手柄数据宽限期: 最后一次收到新数据后, 超过此时间(ms)仍未收到新帧,
// 则自动停止底盘和机械臂, 防止通信中断时执行机构继续运动。
constexpr uint32_t kFreshDataGraceMs = 60;

// ============================================================================
// PID 视觉伺服参数 (机械臂角度轴和前后轴)
// ============================================================================

constexpr float kArmAngleKp = 30.0f;      // 角度轴 比例增益 P
constexpr float kArmAngleKi = 0.5f;       // 角度轴 积分增益 I
constexpr float kArmAngleKd = 5.0f;       // 角度轴 微分增益 D
constexpr float kArmForwardKp = 30.0f;    // 前后轴 比例增益 P
constexpr float kArmForwardKi = 0.5f;     // 前后轴 积分增益 I
constexpr float kArmForwardKd = 5.0f;     // 前后轴 微分增益 D

// ============================================================================
// 上下电机自动序列参数 (按住 A 键时执行)
// 序列: 下降 5 圈 → 夹取等待 5 秒 → 回升到 1 圈
// ============================================================================

constexpr float kDescendTargetRotations = 5.0f;   // 下降目标: 5 圈
constexpr float kAscendTargetRotations = 1.0f;    // 回升目标: 1 圈 (初始位置)
constexpr uint32_t kGripDelayMs = 5000;           // 底部夹取等待时间: 5 秒
constexpr float kVerticalMoveSpeed = 50.0f;       // 上下电机移动速度 (rpm)

// ============================================================================
// 时间估算 (用于无位置反馈时的开环时间控制)

// ============================================================================
constexpr uint32_t kDescentTimeMs = 6000;    // 下降预估时间: 6 秒
constexpr uint32_t kAscentTimeMs = 5000;     // 回升预估时间: 5 秒 
constexpr uint32_t kDescentTimeoutMs = 15000; // 下降超时保护: 15 秒
constexpr uint32_t kAscentTimeoutMs = 12000;  // 回升超时保护: 12 秒

// ============================================================================
// 上下电机状态机
// IDLE(空闲) → DESCENDING(下降中) → GRIPPING(底部夹取等待) → ASCENDING(回升中) → IDLE
// ============================================================================

enum VerticalState {
    IDLE,           // 空闲: 机械臂未在执行上下动作
    DESCENDING,     // 下降中: 电机正转使机械臂向下运动
    GRIPPING,       // 底部夹取等待: 到达目标深度后暂停, 等待夹取完成
    ASCENDING,      // 回升中: 电机反转使机械臂向上运动, 回到初始位置
};

// ============================================================================
// 全局对象指针 (在 setup() 中初始化)
// ============================================================================

ControlSerial* g_control_serial = nullptr;   // 电机控制串口 (Serial2, 地址 1~7)
BottomControl* g_bottom_control = nullptr;   // 底盘控制 (地址 1~4: 四个麦克纳姆轮)
ArmControl* g_arm_control = nullptr;         // 机械臂控制 (地址 5:角度, 6:上下, 7:前后)
VisionSerial* g_vision_serial = nullptr;     // 视觉数据串口 (Serial1, 接收树莓派误差数据)

// ============================================================================
// 全局状态标志
// ============================================================================

bool g_is_stopped = true;          // 底盘当前是否已停止
bool g_arm_active = false;         // 机械臂视觉伺服是否正在运行 (A 键按下时为 true)
bool g_timeout_reported = false;   // 手柄超时警告是否已经输出过 (避免重复打印)
uint32_t g_last_fresh_input_ms = 0; // 最后一次收到有效手柄数据的时间戳

// ============================================================================
// 上下电机状态机变量
// ============================================================================

VerticalState g_vertical_state = IDLE;  // 当前状态机状态
uint32_t g_grip_start_time = 0;         // 进入夹取等待状态的时间戳
uint32_t g_descent_start_time = 0;      // 下降开始时间戳
uint32_t g_ascent_start_time = 0;       // 回升开始时间戳

// ============================================================================
// 安全控制函数
// ============================================================================

/// @brief 停止底盘所有电机 (四个麦克纳姆轮)
void stopChassis() {
    if (g_bottom_control != nullptr) {
        g_bottom_control->stop();
    }
    g_is_stopped = true;
}

/// @brief 停止机械臂所有电机并复位状态机
/// @note 同时停止角度、上下、前后三个电机, 并将上下电机状态机复位为空闲
void stopArm() {
    if (g_arm_control != nullptr) {
        g_arm_control->stop();
    }
    g_vertical_state = IDLE;
    g_arm_active = false;
}

/// @brief 安全停止: 同时停止底盘和机械臂
/// @note 通常在紧急情况或手柄断连/超时时调用
void safetyStopAll() {
    stopChassis();
    stopArm();
}

// ============================================================================
// 底盘控制
// ============================================================================

/// @brief 将手柄解析出的底盘速度命令下发给底盘控制库
/// @param input 手柄输入状态 (包含 vx, vy, wz 三个速度分量)
/// @note vx: 前后速度 mm/s, vy: 左右速度 mm/s, wz: 旋转角速度 rad/s
void applyChassisCommand(const InputState& input) {
    g_bottom_control->motors_control({
        input.chassis.vx_mm_s,
        input.chassis.vy_mm_s,
        input.chassis.wz_rad_s,
    });
    g_is_stopped = false;
}

// ============================================================================
// 机械臂控制
// ============================================================================

/// @brief 机械臂控制主函数
/// @param input 手柄输入状态
/// @details 按住 A 键时同时执行两项控制:
///          1. 视觉伺服 — 角度电机和前后电机根据树莓派发来的误差值做 PID 闭环控制
///          2. 上下电机自动序列 — 下降 5 圈 → 等待 5 秒夹取 → 回升到 1 圈初始位置
///          松开 A 键时立即停止所有机械臂电机并复位状态机
/// @note   当前上下电机使用开环时间估算, 待位置反馈实现后可替换为闭环位置控制
void applyArmCommand(const InputState& input) {
    const uint32_t currentTime = millis();

    // ---- A 键按下: 启用视觉伺服 + 上下电机自动序列 ----
    if (input.armServoTrigger) {
        g_arm_active = true;

        // ================================================================
        // 1. 视觉伺服: 角度 + 前后 PID 闭环控制
        // ================================================================

        // 视觉数据超时提示标志: 每次"从有效变为无效"时仅打印一次警告,
        // 避免串口被重复消息淹没。
        static bool timeout_warned = false;

        // 从视觉串口读取树莓派发来的误差数据
        VisionError visionError = g_vision_serial->read();

        if (visionError.valid) {
            // 数据有效: 清除超时警告标志, 执行 PID 伺服控制
            timeout_warned = false;
            g_arm_control->updateVisualServo(visionError.angleError, visionError.forwardError);

            // 调试输出: 每 200ms 打印一次视觉误差值
            static uint32_t last_debug_print = 0;
            if ((currentTime - last_debug_print) > 200) {
                Serial.printf("Vision: angle=%.2f, forward=%.2f\n",
                              visionError.angleError,
                              visionError.forwardError);
                last_debug_print = currentTime;
            }
        } else {
            // 视觉数据无效 (串口无数据 / 解析失败 / 数据超时):
            // 停止伺服电机并复位 PID 积分项, 防止数据恢复瞬间积分饱和 (windup) 导致窜动
            g_arm_control->stopServo();

            if (!timeout_warned) {
                Serial.println("WARNING: No valid vision data");
                timeout_warned = true;
            }
        }

        // ================================================================
        // 2. 上下电机自动序列状态机
        //    IDLE → DESCENDING → GRIPPING → ASCENDING → IDLE
        // ================================================================

        switch (g_vertical_state) {
            case IDLE: {
                // 首次按下 A 键: 开始下降
                Serial.println("Starting descent to 5 rotations");
                g_vertical_state = DESCENDING;
                g_descent_start_time = currentTime;

                // 新一次伺服开始时复位 PID 状态, 清除上一次遗留的积分值
                g_arm_control->resetPID();

                // TODO: 位置控制协议 (0xFD) 尚未实现, 目前使用速度模式开环控制
                // 将来替换为: g_arm_control->setVerticalPosition(kDescendTargetRotations * 360);
                g_arm_control->manualVertical(-kVerticalMoveSpeed);  // 负值 = 下降
                break;
            }

            case DESCENDING: {
                // 检查是否已到达下降目标位置
                // TODO: 需要位置反馈才能精确判断, 目前使用时间估算
                // 计算: 5 圈 ÷ 50 rpm × 60 = 6 秒
                if ((currentTime - g_descent_start_time) > kDescentTimeMs) {
                    Serial.println("Reached bottom, waiting 5 seconds");
                    g_arm_control->manualVertical(0.0f);  // 停止上下电机
                    g_vertical_state = GRIPPING;
                    g_grip_start_time = currentTime;
                }

                // 下降超时保护: 超过 15 秒强制停止
                if ((currentTime - g_descent_start_time) > kDescentTimeoutMs) {
                    Serial.println("ERROR: Descent timeout! Stopping.");
                    g_arm_control->manualVertical(0.0f);
                    g_vertical_state = IDLE;
                }
                break;
            }

            case GRIPPING: {
                // 在底部等待夹取完成 (默认 5 秒)
                if ((currentTime - g_grip_start_time) >= kGripDelayMs) {
                    Serial.println("Starting ascent to 1 rotation");
                    g_vertical_state = ASCENDING;
                    g_ascent_start_time = currentTime;

                    // TODO: 将来替换为位置控制
                    // g_arm_control->setVerticalPosition(kAscendTargetRotations * 360);
                    g_arm_control->manualVertical(kVerticalMoveSpeed);  // 正值 = 回升
                }
                break;
            }

            case ASCENDING: {
                // 检查是否已回到初始位置
                // TODO: 需要位置反馈, 目前使用时间估算
                // 实际行程: 4 圈 (从 5 圈回到 1 圈), 50rpm 约需 4.8 秒
                if ((currentTime - g_ascent_start_time) > kAscentTimeMs) {
                    Serial.println("Reached top, sequence complete");
                    g_arm_control->manualVertical(0.0f);  // 停止上下电机
                    g_vertical_state = IDLE;               // 回到空闲, 等待下一次触发
                }

                // 回升超时保护: 超过 12 秒强制停止
                if ((currentTime - g_ascent_start_time) > kAscentTimeoutMs) {
                    Serial.println("ERROR: Ascent timeout! Stopping.");
                    g_arm_control->manualVertical(0.0f);
                    g_vertical_state = IDLE;
                }
                break;
            }
        }

    } else {
        // ---- A 键松开: 停止所有机械臂动作并复位 ----
        g_arm_control->stopServo();           // 停止角度和前后伺服电机 + 复位 PID
        g_arm_control->manualVertical(0.0f);  // 停止上下电机
        g_arm_active = false;

        if (g_vertical_state != IDLE) {
            Serial.println("A button released, stopping sequence");
            g_vertical_state = IDLE;          // 复位状态机, 下次按 A 从头开始
        }
    }
}

}  // namespace

// ============================================================================
// Arduino setup() — 上电初始化
// ============================================================================

void setup() {
    // 初始化调试串口 (USB 连接电脑)
    Serial.begin(115200);
    delay(200);
    Serial.println("Chassis and arm control program started");

    // ---- 初始化电机控制串口 (Serial2) ----
    // GPIO16 = RX, GPIO17 = TX
    // 该总线挂载 7 个电机驱动器:
    //   地址 1 ~ 4: 底盘麦克纳姆轮 (左前, 右前, 左后, 右后)
    //   地址 5:     机械臂角度电机
    //   地址 6:     机械臂上下电机
    //   地址 7:     机械臂前后电机
    g_control_serial = new ControlSerial(kMotorBaudRate, kMotorRxPin, kMotorTxPin);

    // 初始化底盘控制 (地址 1~4)
    g_bottom_control = new BottomControl(*g_control_serial);

    // 初始化机械臂控制 (地址 5/6/7, 共用同一 Serial2 总线)
    g_arm_control = new ArmControl(*g_control_serial);

    // ---- 初始化视觉数据串口 (Serial1) ----
    // GPIO18 = RX (接树莓派 TX), GPIO19 = TX (接树莓派 RX)
    // 协议: 0xAA + ID(uint8) + 角度误差(int16 LE) + 前后误差(int16 LE), 共 6 字节
    g_vision_serial = new VisionSerial(kVisionBaudRate, kVisionRxPin, kVisionTxPin);

    // ---- 初始化 Bluepad32 蓝牙手柄输入层 ----
    GamepadInput::begin();

    // ---- 上电安全: 先停止所有执行机构, 避免上电瞬间误动作 ----
    stopChassis();
    stopArm();

    // ---- 配置机械臂 PID 参数 ----
    // 角度轴和前后轴使用相同的 PID 增益, 可根据实际调试效果分别调整
    g_arm_control->setAnglePID(kArmAngleKp, kArmAngleKi, kArmAngleKd);
    g_arm_control->setForwardPID(kArmForwardKp, kArmForwardKi, kArmForwardKd);
    Serial.println("PID parameters: Kp=30.0, Ki=0.5, Kd=5.0");

    Serial.println("Initialization complete");
}

// ============================================================================
// Arduino loop() — 主循环 (约 50Hz, 每 20ms 执行一次)
// ============================================================================

void loop() {
    // Bluepad32 必须每帧调用 update(), 否则手柄状态不会刷新
    GamepadInput::update();

    // 读取当前帧封装好的输入状态 (手柄连接/断开/超时/摇杆/按键 等)
    const InputState input = GamepadInput::read();

    // ---- 情况 1: 无可用手柄 ----
    // 没有手柄连接时, 底盘和机械臂必须保持停止, 不允许任何运动
    if (!input.connected) {
        if (!g_is_stopped || g_arm_active) {
            safetyStopAll();
            Serial.println("No active controller, chassis and arm stopped");
        }
        g_timeout_reported = false;
        delay(20);
        return;
    }

    // ---- 情况 2: 手柄已连接但数据超时 ----
    // 手柄物理连接正常, 但超过 250ms 没有收到新数据包 (蓝牙断流),
    // 此时也必须停止所有运动, 避免执行机构在通信中断期间继续执行过时指令
    if (input.timedOut) {
        if (!g_is_stopped || g_arm_active) {
            safetyStopAll();
        }
        if (!g_timeout_reported) {
            Serial.println("Controller data timeout, chassis and arm stopped");
            g_timeout_reported = true;  // 只打印一次, 避免串口刷屏
        }
        delay(20);
        return;
    }

    // 手柄数据恢复正常, 清除超时报告标志
    g_timeout_reported = false;

    // ---- 情况 3: 有有效手柄连接且数据未超时 ----
    // hasFreshData == true:  当前帧有新的手柄数据 → 下发控制指令
    // hasFreshData == false: 当前帧无新数据但未超时 → 宽限期后清零停止,
    //                        防止沿用上一帧的速度值继续运动
    if (input.hasFreshData) {
        g_last_fresh_input_ms = input.timestampMs;
        applyChassisCommand(input);   // 更新底盘速度
        applyArmCommand(input);       // 更新机械臂控制 (视觉伺服 + 上下序列)
    } else if ((!g_is_stopped || g_arm_active) &&
               (input.timestampMs - g_last_fresh_input_ms) > kFreshDataGraceMs) {
        // 超过宽限期仍无新数据: 同时停止底盘和机械臂
        safetyStopAll();
    }

    // 主循环延迟 20ms, 控制频率约 50Hz
    delay(20);
}
