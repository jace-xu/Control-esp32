#include <Arduino.h>
#include <BottomControl.h>
#include <ArmControl.h>
#include <ArmSequence.h>
#include <TrayControl.h>
#include <TaskCoordinator.h>
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
// 全局对象指针 (在 setup() 中初始化)
// ============================================================================

BottomControl* g_bottom_control = nullptr;   // 底盘控制 (地址 1~4: 四个麦克纳姆轮, 单例)
ArmControl* g_arm_control = nullptr;         // 机械臂控制 (地址 5:角度, 6:上下, 7:前后)
VisionSerial* g_vision_serial = nullptr;     // 视觉数据串口 (Serial1, 接收树莓派误差数据)
ArmSequence* g_arm_sequence = nullptr;       // 机械臂任务编排 (视觉伺服 + 上下序列状态机)
TrayControl* g_tray_control = nullptr;       // 物料盘控制 (骨架, 动作待补)
TaskCoordinator* g_task_coordinator = nullptr; // 上层协调器 (机械臂 + 物料盘; 当前透传)

// ============================================================================
// 全局状态标志
// ============================================================================

bool g_is_stopped = true;          // 底盘当前是否已停止
bool g_timeout_reported = false;   // 手柄超时警告是否已经输出过 (避免重复打印)
uint32_t g_last_fresh_input_ms = 0; // 最后一次收到有效手柄数据的时间戳

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
/// @note 委托给协调器 (机械臂 + 未来物料盘); 协调器未就绪时回退到 ArmSequence
void stopArm() {
    if (g_task_coordinator != nullptr) {
        g_task_coordinator->stop();
    } else if (g_arm_sequence != nullptr) {
        g_arm_sequence->stop();
    }
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

/// @brief 将 A/X/B 三键状态转发给机械臂任务编排器
/// @param input 手柄输入状态
/// @note  两阶段 + 颜色队列模式:
///        - A 键(阶段1): 请求误差1, 握手后记录一个物料颜色入队, 跑完整升降序列。每按一次记一个。
///        - X 键(阶段2): 请求误差2, 按队列 FIFO 逐个颜色自动跑完所有升降序列。
///        - B 键: 中止当前运行, 复位为 IDLE (队列保留)。
///        - L1 键(阶段1前置): 空闲时角度电机预备摆位 (PRE_CATCH), 再按 A 进对准。
///        上升沿触发, 序列自锁运行; A/X/B/L1 的边沿检测与队列逻辑封装在 ArmSequence。
void applyArmCommand(const InputState& input) {
    g_task_coordinator->update(input.buttons.a, input.buttons.x, input.buttons.b, input.buttons.l1);
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
    ControlSerial::get_instance().initialize(kMotorBaudRate, kMotorRxPin, kMotorTxPin);

    // 初始化底盘控制 (地址 1~4, 单例)
    g_bottom_control = &(BottomControl::get_instance());

    // ---- 初始化视觉数据串口 (Serial1) ----
    // GPIO18 = RX (接树莓派 TX), GPIO19 = TX (接树莓派 RX)
    // 协议: 0xAA + ID1(uint8) + ID2(uint8) + 角度误差(int16 LE) + 前后误差(int16 LE), 共 8 字节
    // [ ! ] 须在 ArmControl 之前创建: ArmControl 集成视觉对准, 构造需传入 VisionSerial 指针
    g_vision_serial = new VisionSerial(kVisionBaudRate, kVisionRxPin, kVisionTxPin);

    // 初始化机械臂控制 (地址 5/6/7, 共用同一 Serial2 总线; 集成视觉对准, 依赖 VisionSerial)
    g_arm_control = new ArmControl(g_vision_serial);

    // 初始化机械臂任务编排 (依赖 ArmControl, 须在其之后创建)
    g_arm_sequence = new ArmSequence(g_arm_control);

    // 初始化物料盘控制 (骨架; 与底盘/机械臂共用 Serial2 总线)
    g_tray_control = new TrayControl();

    // 初始化上层协调器 (依赖 ArmSequence 与 TrayControl, 须在二者之后创建)
    g_task_coordinator = new TaskCoordinator(g_arm_sequence, g_tray_control);

    // ---- 初始化 Bluepad32 蓝牙手柄输入层 ----
    GamepadInput::begin();

    // ---- 上电安全: 先停止所有执行机构, 避免上电瞬间误动作 ----
    stopChassis();
    stopArm();

    // 机械臂 PID 增益的权威默认值定义在 ArmControl 内 (现场标定值);
    // 如需运行时改, 在此调用 g_arm_control->setAnglePID/setForwardPID 覆盖。

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
        if (!g_is_stopped || g_task_coordinator->isActive()) {
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
        if (!g_is_stopped || g_task_coordinator->isActive()) {
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
        delay(2);  // 底盘指令下发后短暂延迟, 确保 Serial2 总线有时间处理指令, 再下发机械臂指令
        
        applyArmCommand(input);       // 更新机械臂控制 (视觉伺服 + 上下序列)
    } else if ((!g_is_stopped || g_task_coordinator->isActive()) &&
               (input.timestampMs - g_last_fresh_input_ms) > kFreshDataGraceMs) {
        // 超过宽限期仍无新数据: 同时停止底盘和机械臂
        safetyStopAll();
    }

    // 主循环延迟 20ms, 控制频率约 50Hz
    delay(20);
}
