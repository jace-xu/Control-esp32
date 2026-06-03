#pragma once
#include <Arduino.h>
#include <ArmControl.h>
#include <VisionSerial.h>

/**
 * @brief 机械臂高层任务编排: 视觉伺服 + 上下电机自动序列
 * @note  封装原 main.cpp 中的 applyArmCommand 逻辑, 把状态机和相关状态收归自身,
 *        让 main 只需每帧调用 update() 即可。
 * @note  依赖注入: 构造时传入已初始化好的 ArmControl 与 VisionSerial 指针,
 *        本类不拥有它们的生命周期 (不负责 delete)。
 * @note  == Version 1.0.0 ==
 */
class ArmSequence {
private:
    // ========================================================================
    // 上下电机状态机
    // IDLE(空闲) → DESCENDING(下降中) → GRIPPING(底部夹取等待) → ASCENDING(回升中) → IDLE
    // ========================================================================
    enum VerticalState {
        IDLE,        // 空闲: 机械臂未在执行上下动作
        DESCENDING,  // 下降中: 电机正转使机械臂向下运动
        GRIPPING,    // 底部夹取等待: 到达目标深度后暂停, 等待夹取完成
        ASCENDING,   // 回升中: 电机反转使机械臂向上运动, 回到初始位置
    };

    // ========================================================================
    // 上下电机自动序列参数 (按住 A 键时执行)
    // 序列: 下降 5 圈 → 夹取等待 5 秒 → 回升到 1 圈
    // ========================================================================
    static constexpr float kDescendTargetRotations = 5.0f;   // 下降目标: 5 圈 (位置控制实现后使用)
    static constexpr float kAscendTargetRotations = 1.0f;    // 回升目标: 1 圈 (位置控制实现后使用)
    static constexpr uint32_t kGripDelayMs = 5000;           // 底部夹取等待时间: 5 秒
    static constexpr float kVerticalMoveSpeed = 50.0f;       // 上下电机移动速度 (rpm)

    // 时间估算 (用于无位置反馈时的开环时间控制)
    static constexpr uint32_t kDescentTimeMs = 6000;     // 下降预估时间: 6 秒
    static constexpr uint32_t kAscentTimeMs = 5000;      // 回升预估时间: 5 秒
    static constexpr uint32_t kDescentTimeoutMs = 15000; // 下降超时保护: 15 秒
    static constexpr uint32_t kAscentTimeoutMs = 12000;  // 回升超时保护: 12 秒

    // ========================================================================
    // 协作对象 (不拥有生命周期)
    // ========================================================================
    ArmControl* arm = nullptr;        // 机械臂电机控制 (地址 5/6/7)
    VisionSerial* vision = nullptr;   // 视觉数据串口 (接收树莓派误差)

    // ========================================================================
    // 运行时状态
    // ========================================================================
    bool active = false;                    // 视觉伺服是否正在运行 (A 键按下时为 true)
    VerticalState state = IDLE;             // 当前状态机状态
    uint32_t grip_start_time = 0;           // 进入夹取等待状态的时间戳
    uint32_t descent_start_time = 0;        // 下降开始时间戳
    uint32_t ascent_start_time = 0;         // 回升开始时间戳

    // 视觉伺服内部标志 (原为函数内 static 变量)
    bool timeout_warned = false;            // "无有效视觉数据" 警告是否已打印过
    uint32_t last_debug_print = 0;          // 上次打印视觉误差调试信息的时间戳

public:
    // 禁用拷贝和赋值
    ArmSequence(const ArmSequence&) = delete;
    ArmSequence& operator=(const ArmSequence&) = delete;

    /**
     * @brief 构造机械臂任务编排器
     * @param arm    已初始化的机械臂控制对象指针
     * @param vision 已初始化的视觉串口对象指针
     */
    ArmSequence(ArmControl* arm, VisionSerial* vision)
        : arm(arm), vision(vision) {}

    /// @brief 视觉伺服是否正在运行 (供 main 做安全判断)
    bool isActive() const { return active; }

    /**
     * @brief 每帧调用的主入口
     * @param triggerPressed A 键是否按住
     * @details 按住: 执行视觉伺服 + 上下电机自动序列; 松开: 停止机械臂动作并复位。
     */
    void update(bool triggerPressed) {
        const uint32_t currentTime = millis();

        if (triggerPressed) {
            active = true;
            runVisualServo(currentTime);
            runVerticalSequence(currentTime);
        } else {
            handleRelease();
        }
    }

    /**
     * @brief 立即停止所有机械臂电机并复位状态机
     * @note  紧急停止 / 手柄断连超时时由 main 调用。
     */
    void stop() {
        if (arm != nullptr) {
            arm->stop();
        }
        state = IDLE;
        active = false;
    }

private:
    /**
     * @brief 视觉伺服: 角度 + 前后 PID 闭环控制
     * @param currentTime 当前 millis() 时间戳
     */
    void runVisualServo(uint32_t currentTime) {
        // 从视觉串口读取树莓派发来的误差数据
        VisionError visionError = vision->read();

        if (visionError.valid) {
            // 数据有效: 清除超时警告标志, 执行 PID 伺服控制
            timeout_warned = false;
            arm->updateVisualServo(visionError.angleError, visionError.forwardError);

            // 调试输出: 每 200ms 打印一次视觉误差值
            if ((currentTime - last_debug_print) > 200) {
                Serial.printf("Vision: angle=%.2f, forward=%.2f\n",
                              visionError.angleError,
                              visionError.forwardError);
                last_debug_print = currentTime;
            }
        } else {
            // 视觉数据无效 (串口无数据 / 解析失败 / 数据超时):
            // 停止伺服电机并复位 PID 积分项, 防止数据恢复瞬间积分饱和 (windup) 导致窜动
            arm->stopServo();

            if (!timeout_warned) {
                Serial.println("WARNING: No valid vision data");
                timeout_warned = true;
            }
        }
    }

    /**
     * @brief 上下电机自动序列状态机
     *        IDLE → DESCENDING → GRIPPING → ASCENDING → IDLE
     * @param currentTime 当前 millis() 时间戳
     */
    void runVerticalSequence(uint32_t currentTime) {
        switch (state) {
            case IDLE: {
                // 首次按下 A 键: 开始下降
                Serial.println("Starting descent to 5 rotations");
                state = DESCENDING;
                descent_start_time = currentTime;

                // 新一次伺服开始时复位 PID 状态, 清除上一次遗留的积分值
                arm->resetPID();

                // TODO: 位置控制协议 (0xFD) 尚未实现, 目前使用速度模式开环控制
                // 将来替换为: arm->setVerticalPosition(kDescendTargetRotations * 360);
                arm->manualVertical(-kVerticalMoveSpeed);  // 负值 = 下降
                break;
            }

            case DESCENDING: {
                // 检查是否已到达下降目标位置
                // TODO: 需要位置反馈才能精确判断, 目前使用时间估算
                // 计算: 5 圈 ÷ 50 rpm × 60 = 6 秒
                if ((currentTime - descent_start_time) > kDescentTimeMs) {
                    Serial.println("Reached bottom, waiting 5 seconds");
                    arm->manualVertical(0.0f);  // 停止上下电机
                    state = GRIPPING;
                    grip_start_time = currentTime;
                }

                // 下降超时保护: 超过 15 秒强制停止
                if ((currentTime - descent_start_time) > kDescentTimeoutMs) {
                    Serial.println("ERROR: Descent timeout! Stopping.");
                    arm->manualVertical(0.0f);
                    state = IDLE;
                }
                break;
            }

            case GRIPPING: {
                // 在底部等待夹取完成 (默认 5 秒)
                if ((currentTime - grip_start_time) >= kGripDelayMs) {
                    Serial.println("Starting ascent to 1 rotation");
                    state = ASCENDING;
                    ascent_start_time = currentTime;

                    // TODO: 将来替换为位置控制
                    // arm->setVerticalPosition(kAscendTargetRotations * 360);
                    arm->manualVertical(kVerticalMoveSpeed);  // 正值 = 回升
                }
                break;
            }

            case ASCENDING: {
                // 检查是否已回到初始位置
                // TODO: 需要位置反馈, 目前使用时间估算
                // 实际行程: 4 圈 (从 5 圈回到 1 圈), 50rpm 约需 4.8 秒
                if ((currentTime - ascent_start_time) > kAscentTimeMs) {
                    Serial.println("Reached top, sequence complete");
                    arm->manualVertical(0.0f);  // 停止上下电机
                    state = IDLE;               // 回到空闲, 等待下一次触发
                }

                // 回升超时保护: 超过 12 秒强制停止
                if ((currentTime - ascent_start_time) > kAscentTimeoutMs) {
                    Serial.println("ERROR: Ascent timeout! Stopping.");
                    arm->manualVertical(0.0f);
                    state = IDLE;
                }
                break;
            }
        }
    }

    /**
     * @brief A 键松开: 停止所有机械臂动作并复位状态机
     */
    void handleRelease() {
        arm->stopServo();           // 停止角度和前后伺服电机 + 复位 PID
        arm->manualVertical(0.0f);  // 停止上下电机
        active = false;

        if (state != IDLE) {
            Serial.println("A button released, stopping sequence");
            state = IDLE;           // 复位状态机, 下次按 A 从头开始
        }
    }
};
