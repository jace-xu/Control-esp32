#pragma once
#include <Arduino.h>
#include <ArmControl.h>
#include <ColorQueue.h>

/**
 * @brief 阶段2 放置序列: 从物料盘取料 → 折回 → 放置 (X 键触发, 按颜色队列 FIFO 消费)
 * @note  从 ArmSequence 拆出的独立状态机。视觉对准复用 ArmControl 集成的对准接口
 *        (beginAlign/updateAlign/...); 本类只负责阶段2 的步骤编排与位置轮询。
 * @note  依赖注入: 构造传入 ArmControl 与 ColorQueue 指针 (颜色队列由 ArmSequence 拥有,
 *        阶段1 入队、阶段2 出队), 本类不拥有其生命周期。
 * @note  流程: 按X取队首色 → 先降一小段+PID对准 → 收敛后自动记录前后/角度位 →
 *        移到固定取料位 → 下降夹取 → 上升并折回对准位 → 按X下降放置位 → 再按X松爪 →
 *        队列非空则自动取下一色。两个 X 均为人工确认门 (不重对准)。
 * @note  == Version 1.0.0 ==
 */
class PlaceSequence {
private:
    // ========================================================================
    // 阶段2 状态机
    // ========================================================================
    enum State {
        IDLE,            // 空闲
        PRE_ALIGN,       // 先降一小段 + PID 对准; 收敛后自动推进 (不等X)
        MOVE_TO_PICKUP,  // 角度+前后 → 固定取料位, 轮询两轴到位
        DESCEND_PICKUP,  // 上下 → 取料深度, 到位 grip()
        GRIP_SETTLE,     // 等舵机夹紧 settle
        ASCEND_RETURN,   // 上下回升 + 前后/角度折回记录位, 轮询三轴到位
        WAIT_X_DOWN,     // 人工确认门: 等 X → 下降到放置位
        DESCEND_PLACE,   // 上下 → 放置位, 轮询到位
        WAIT_X_RELEASE,  // 人工确认门: 等 X → release() → 本色完成
    };

    // ========================================================================
    // 参数 (圈数带符号: 沿用 ArmSequence 的 kVerticalDirection/kRotationDirection 约定;
    //       正=下降/逆时针。占位值现场校准。)
    // ========================================================================
    static constexpr float kVerticalDirection = 1.0f;  // 上下方向因子: 正=下降
    static constexpr float kRotationDirection = 1.0f;  // 旋转方向因子: 正=逆时针

    static constexpr float kPreDescendRotations = 0.5f*kVerticalDirection;   // 先降一小段
    static constexpr float kPickupForwardDeg = 0.0f;                          // 固定取料位: 前后绝对角 (占位)
    static constexpr float kPickupAngleDeg = 0.0f*kRotationDirection;         // 固定取料位: 角度绝对角 (占位)
    static constexpr float kPickupDescendRotations = 5.0f*kVerticalDirection; // 下降到取料深度: 5 圈 (占位)
    static constexpr float kReturnAscendRotations = 1.0f*kVerticalDirection;  // 折回时上下回升圈数 (占位)
    static constexpr float kPlaceDescendRotations = 5.0f*kVerticalDirection;  // 下降到放置位: 5 圈 (占位)

    static constexpr uint32_t kGripSettleMs = 800;          // 舵机夹取到位等待时间
    static constexpr float kPositionArrivedThreshDeg = 5.0f;// 位置到位判定阈值 (度)
    static constexpr uint32_t kPreAlignTimeoutMs = 30000;   // PRE_ALIGN 等待收敛的超时 (30秒)
    static constexpr uint32_t kDescentTimeoutMs = 15000;    // 下降超时保护: 15 秒
    static constexpr uint32_t kAscentTimeoutMs = 12000;     // 回升/移动超时保护: 12 秒

    // ========================================================================
    // 协作对象 (不拥有生命周期)
    // ========================================================================
    ArmControl* arm = nullptr;        // 机械臂电机控制 + 集成视觉对准
    ColorQueue* queue = nullptr;      // 物料颜色队列 (ArmSequence 拥有; 本类出队)

    // ========================================================================
    // 运行时状态
    // ========================================================================
    bool active = false;              // 序列是否正在运行
    State state = IDLE;               // 当前状态
    bool servo_stopped = false;       // 非串流位置控制期是否已停一次伺服

    uint32_t step_start_time = 0;     // 当前步骤开始时间戳 (超时基准)
    uint32_t grip_start_time = 0;     // 进入 GRIP_SETTLE 的时间戳

    // 位置目标 (轮询到位判定)
    float vertical_target_degrees = 0.0f;  // 上下电机目标角度
    float forward_target_degrees = 0.0f;   // 前后电机目标角度
    float angle_target_degrees = 0.0f;     // 角度电机目标角度

    // 第2步收敛瞬间记录的"对准位" (= 第5步折回目标)
    float recorded_forward_deg = 0.0f;     // 收敛时前后电机位置
    float recorded_angle_deg = 0.0f;       // 收敛时角度电机位置

public:
    // 禁用拷贝和赋值
    PlaceSequence(const PlaceSequence&) = delete;
    PlaceSequence& operator=(const PlaceSequence&) = delete;

    /**
     * @brief 构造阶段2 放置序列
     * @param arm   已初始化的机械臂控制对象指针 (含视觉对准)
     * @param queue 颜色队列指针 (ArmSequence 拥有; 本类出队消费)
     */
    PlaceSequence(ArmControl* arm, ColorQueue* queue) : arm(arm), queue(queue) {}

    /// @brief 序列是否正在运行
    bool isActive() const { return active; }

    /**
     * @brief 尝试启动阶段2 (X 键上升沿, 仅空闲时): 从队列取首色, 队空则不启动
     * @param currentTime 当前 millis() 时间戳
     * @return true=已启动; false=队空未启动
     */
    bool tryStart(uint32_t currentTime) {
        uint8_t color = 0;
        if (queue == nullptr || !queue->dequeue(color)) {
            Serial.println("[Stage 2] Place: color queue empty, nothing to do");
            return false;
        }
        Serial.printf("[Stage 2] Place: requesting error2 for color %u (%u left)\n",
                      color, queue->count());
        begin(currentTime, color);
        return true;
    }

    /**
     * @brief 每帧推进 (仅在 active 时由 ArmSequence 调用)
     * @param currentTime  当前 millis() 时间戳
     * @param xRising      X 键上升沿 (人工确认门: WAIT_X_DOWN / WAIT_X_RELEASE)
     * @param abortPressed B 键: 中止当前运行并复位 (松爪)
     */
    void update(uint32_t currentTime, bool xRising, bool abortPressed) {
        if (!active) {
            return;
        }

        // 中止: 松爪 + 停止复位 (颜色队列保留, 已出队的本色视作丢失)
        if (abortPressed) {
            Serial.println("[Stage 2] Aborted by user");
            if (arm != nullptr) {
                arm->release();
            }
            haltAndReset();
            return;
        }

        // 人工确认门: X 上升沿推进
        if (xRising) {
            if (state == WAIT_X_DOWN) {
                Serial.println("[Stage 2] X: descending to place position");
                vertical_target_degrees = kPlaceDescendRotations * 360.0f;
                arm->setVerticalPosition(vertical_target_degrees);
                state = DESCEND_PLACE;
                step_start_time = currentTime;
            } else if (state == WAIT_X_RELEASE) {
                Serial.println("[Stage 2] X: releasing material");
                arm->release();
                finishRun(currentTime);
                return;
            }
        }

        // 串流期跑视觉对准; 非串流位置控制期停一次伺服
        if (arm->isStreaming()) {
            arm->retryAlignIfNeeded(currentTime);
            runAlign(currentTime);
            servo_stopped = false;
        } else if (!servo_stopped && state != PRE_ALIGN) {
            arm->stopServo();
            servo_stopped = true;
        }

        runStateMachine(currentTime);
    }

    /// @brief 立即停止并复位 (中止/急停/手柄断连时调用)
    void stop() { haltAndReset(); }

private:
    // ========================================================================
    // 停止原语
    // ========================================================================

    /// @brief 停所有臂电机 + 停视觉串流 (不动状态机)
    void haltMotors() {
        arm->stopAlignStream();   // 通知树莓派停止发误差
        arm->stop();              // 停三个电机并复位 PID
    }

    /// @brief 复位状态机与自锁标志 (回空闲, 不碰电机/队列)
    void resetRunState() {
        state = IDLE;
        active = false;
        servo_stopped = false;
    }

    /// @brief 彻底停止: 停电机 + 复位状态机
    void haltAndReset() {
        haltMotors();
        resetRunState();
    }

    // ========================================================================
    // 启动 / 推进
    // ========================================================================

    /**
     * @brief 启动一次放置序列: 开视觉对准 + 先降一小段 + 进 PRE_ALIGN
     * @param currentTime 当前 millis() 时间戳
     * @param color       本色 (误差2 目标颜色)
     */
    void begin(uint32_t currentTime, uint8_t color) {
        active = true;
        servo_stopped = false;
        arm->beginAlign(2, color, currentTime);   // 请求误差2 + resetPID + 收敛清零
        Serial.printf("[Stage 2] Pre-descend %.2f rotations, aligning...\n", kPreDescendRotations);
        vertical_target_degrees = kPreDescendRotations * 360.0f;  // 符号已在常量里
        arm->setVerticalPosition(vertical_target_degrees);
        step_start_time = currentTime;
        state = PRE_ALIGN;
    }

    /**
     * @brief PRE_ALIGN 期视觉伺服一帧; 收敛后自动记录对准位并进 MOVE_TO_PICKUP
     * @param currentTime 当前 millis() 时间戳
     * @note  阶段2 收敛即自动推进 (不等X), 与阶段1 二次按A 确认不同。
     */
    void runAlign(uint32_t currentTime) {
        ArmControl::AlignFrame f = arm->updateAlign(currentTime);
        if (state == PRE_ALIGN && f.converged) {
            // 收敛瞬间记录当前前后 + 角度位置 (= 折回目标)
            arm->readForwardPosition(recorded_forward_deg);
            arm->readAnglePosition(recorded_angle_deg);
            Serial.printf("[Stage 2] Aligned! recorded forward=%.1f angle=%.1f, moving to pickup\n",
                          recorded_forward_deg, recorded_angle_deg);

            // 停视觉伺服, 转位置控制: 移动角度+前后到固定取料位
            arm->stopAlignStream();
            arm->stopServo();
            arm->setCountConvergence(false);
            forward_target_degrees = kPickupForwardDeg;
            angle_target_degrees = kPickupAngleDeg;
            arm->setForwardPosition(kPickupForwardDeg);
            arm->setAnglePosition(kPickupAngleDeg);
            state = MOVE_TO_PICKUP;
            step_start_time = currentTime;
        }
    }

    /// @brief 两轴/三轴到位判定辅助 (读位置且在阈值内)
    bool forwardArrived() {
        float p = 0.0f;
        return arm->readForwardPosition(p) &&
               fabsf(p - forward_target_degrees) <= kPositionArrivedThreshDeg;
    }
    bool angleArrived() {
        float p = 0.0f;
        return arm->readAnglePosition(p) &&
               fabsf(p - angle_target_degrees) <= kPositionArrivedThreshDeg;
    }
    bool verticalArrived() {
        float p = 0.0f;
        return arm->readVerticalPosition(p) &&
               fabsf(p - vertical_target_degrees) <= kPositionArrivedThreshDeg;
    }

    /**
     * @brief 阶段2 状态机推进 (位置轮询 + 超时保护)
     * @param currentTime 当前 millis() 时间戳
     */
    void runStateMachine(uint32_t currentTime) {
        switch (state) {
            case IDLE:
                break;

            // ---- PRE_ALIGN: 对准在 runAlign 推进; 此处仅超时防护 ----
            case PRE_ALIGN:
                if ((currentTime - step_start_time) > kPreAlignTimeoutMs) {
                    Serial.println("ERROR: [Stage 2] Pre-align timeout (no convergence)! Stopping.");
                    haltAndReset();
                }
                break;

            // ---- MOVE_TO_PICKUP: 角度+前后到固定取料位, 两轴到位 → 下降取料 ----
            case MOVE_TO_PICKUP:
                if (forwardArrived() && angleArrived()) {
                    Serial.println("[Stage 2] At pickup pose, descending to grip");
                    vertical_target_degrees = kPickupDescendRotations * 360.0f;
                    arm->setVerticalPosition(vertical_target_degrees);
                    state = DESCEND_PICKUP;
                    step_start_time = currentTime;
                }
                if ((currentTime - step_start_time) > kAscentTimeoutMs) {
                    Serial.println("ERROR: [Stage 2] Move-to-pickup timeout! Stopping.");
                    haltAndReset();
                }
                break;

            // ---- DESCEND_PICKUP: 上下到取料深度, 到位 grip() ----
            case DESCEND_PICKUP:
                if (verticalArrived()) {
                    Serial.println("[Stage 2] At pickup depth, gripping");
                    arm->grip();
                    state = GRIP_SETTLE;
                    grip_start_time = currentTime;
                }
                if ((currentTime - step_start_time) > kDescentTimeoutMs) {
                    Serial.println("ERROR: [Stage 2] Descend-pickup timeout! Stopping.");
                    haltAndReset();
                }
                break;

            // ---- GRIP_SETTLE: 等舵机夹紧, 之后上升并折回对准位 ----
            case GRIP_SETTLE:
                if ((currentTime - grip_start_time) >= kGripSettleMs) {
                    Serial.println("[Stage 2] Gripped, ascending and returning to aligned pose");
                    vertical_target_degrees = kReturnAscendRotations * 360.0f;
                    forward_target_degrees = recorded_forward_deg;
                    angle_target_degrees = recorded_angle_deg;
                    arm->setVerticalPosition(vertical_target_degrees);
                    arm->setForwardPosition(recorded_forward_deg);
                    arm->setAnglePosition(recorded_angle_deg);
                    state = ASCEND_RETURN;
                    step_start_time = currentTime;
                }
                break;

            // ---- ASCEND_RETURN: 上下回升 + 前后/角度折回, 三轴到位 → 等X下降 ----
            case ASCEND_RETURN:
                if (verticalArrived() && forwardArrived() && angleArrived()) {
                    Serial.println("[Stage 2] Returned to aligned pose. Press X to descend & place.");
                    state = WAIT_X_DOWN;
                    step_start_time = currentTime;
                }
                if ((currentTime - step_start_time) > kAscentTimeoutMs) {
                    Serial.println("ERROR: [Stage 2] Ascend-return timeout! Stopping.");
                    haltAndReset();
                }
                break;

            // ---- WAIT_X_DOWN: 人工确认门, X 推进由 update() 处理; 无超时 (等人) ----
            case WAIT_X_DOWN:
                break;

            // ---- DESCEND_PLACE: 上下到放置位, 到位 → 等X松爪 ----
            case DESCEND_PLACE:
                if (verticalArrived()) {
                    Serial.println("[Stage 2] At place position. Press X to release.");
                    state = WAIT_X_RELEASE;
                    step_start_time = currentTime;
                }
                if ((currentTime - step_start_time) > kDescentTimeoutMs) {
                    Serial.println("ERROR: [Stage 2] Descend-place timeout! Stopping.");
                    haltAndReset();
                }
                break;

            // ---- WAIT_X_RELEASE: 人工确认门, X 推进由 update() 处理; 无超时 (等人) ----
            case WAIT_X_RELEASE:
                break;
        }
    }

    /**
     * @brief 本色完成后推进: 队列还有色 → 自动续跑; 队空 → 结束
     * @param currentTime 当前 millis() 时间戳
     */
    void finishRun(uint32_t currentTime) {
        haltMotors();  // 停本色的串流与电机
        uint8_t color = 0;
        if (queue != nullptr && queue->dequeue(color)) {
            Serial.printf("[Stage 2] Next color %u (%u left)\n", color, queue->count());
            begin(currentTime, color);   // 续跑 (自锁继续)
            return;
        }
        Serial.println("[Stage 2] Place: queue drained, all done");
        resetRunState();
    }
};
