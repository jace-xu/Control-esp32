#pragma once
#include <Arduino.h>
#include <ArmControl.h>
#include <VisionSerial.h>
#include <ColorQueue.h>
#include <PlaceSequence.h>

/**
 * @brief 机械臂高层任务编排: 阶段1(夹取) + 委托阶段2(放置) + 颜色队列
 * @note  阶段1 (A键夹取) 编排留在本类; 阶段2 (X键放置) 拆到 PlaceSequence。
 *        视觉对准复用 ArmControl 集成的对准接口 (beginAlign/updateAlign/...),
 *        本类不再直接持有 VisionSerial/握手。
 * @note  依赖注入: 构造传入已初始化的 ArmControl 指针, 本类不拥有其生命周期。
 * @note  对外接口不变: main/TaskCoordinator 仍调 update(a,x,b,l1)。
 * @note  == Version 3.0.0 ==
 */
class ArmSequence {
private:
    // ========================================================================
    // 上下电机状态机 (阶段1: A键夹取, 收臂归位)
    //   [PRE_CATCH(L1预备摆位)] → PRE_ALIGN(先降一小段+PID对准,等A确认)
    //   → DESCENDING(位置下降到夹取位) → GRIPPING(舵机夹取+settle)
    //   → STOW_LIFT_FORWARD(上下+前后归位) → STOW_ANGLE(角度归位) → STOW_DESCEND(上下再降)
    //   → release放料 → IDLE
    // 阶段2 (X键放置) 已拆到 PlaceSequence, 本类只在空闲时转发启动 + 运行期委托。
    // ========================================================================
    enum VerticalState {
        IDLE,              // 空闲: 机械臂未在执行上下动作
        PRE_CATCH,         // [前置] 预备摆位: 按 L1 后角度电机转到 kPreCatchAngleDeg, 等按 A
        PRE_ALIGN,         // 对准等待: 上下已先降一小段, 角度+前后跑PID对准, 等第二次按 A
        DESCENDING,        // 下降中: 位置控制到夹取位
        GRIPPING,          // 底部动作: 舵机夹取 + settle
        STOW_LIFT_FORWARD, // [收臂] 步骤1: 上下+前后电机同时归位, 轮询两轴到位
        STOW_ANGLE,        // [收臂] 步骤2: 角度电机归位, 轮询到位
        STOW_DESCEND,      // [收臂] 步骤3: 上下电机再下降, 到位后 release 放料
    };

    // ========================================================================
    // 阶段1 上下电机自动序列参数
    // [ ! ] 圈数带符号: 正=下降约定。setPosition 原样下发, 方向反了改这里符号即可。
    //       目标绝对角度 = 圈数常量 × 360。
    // ========================================================================
    static constexpr float kVerticalDirection = -1.0f;  // 上下方向因子: 负=下降, 正=上升
    static constexpr float kRotationDirection = -1.0f;  // 旋转方向因子: 正=逆时针, 负=顺时针

    static constexpr float kDescendTargetRotations = 5.0f*kVerticalDirection;  // 下降到夹取位: 5 圈

    // 夹取后收臂归位目标 (占位, 现场校准; 圈数带符号同方向因子约定)
    static constexpr float kStowLiftRotations = 1.0f*kVerticalDirection;   // 步骤1 上下电机归位圈数
    static constexpr float kStowForwardDeg = 0.0f;                          // 步骤1 前后电机归位绝对角 (占位)
    static constexpr float kStowAngleDeg = 0.0f;                            // 步骤2 角度电机归位绝对角 (占位)
    static constexpr float kFinalDescendRotations = 3.0f*kVerticalDirection;// 步骤3 上下电机再下降圈数 (放料高度)

    // 超时保护 (位置反馈失败/电机卡死时强制停止)
    static constexpr uint32_t kDescentTimeoutMs = 15000; // 下降超时保护: 15 秒
    static constexpr uint32_t kAscentTimeoutMs = 12000;  // 回升/归位超时保护: 12 秒

    // 位置控制参数
    static constexpr float kPreDescendRotations = 0.0f*kVerticalDirection;    // 第一次按A先降0.5圈
    static constexpr uint32_t kGripSettleMs = 800;          // 舵机夹取到位等待时间
    static constexpr float kPositionArrivedThreshDeg = 5.0f;// 位置到位判定阈值 (度)
    static constexpr uint32_t kPreAlignTimeoutMs = 30000;   // PRE_ALIGN 等待第二次按A的超时 (30秒)

    // PRE_CATCH 预备摆位参数 (前置, 按 L1 触发)
    static constexpr float kPreCatchAngleDeg = 180.0f*kRotationDirection;       // 角度电机预备摆位目标绝对角
    static constexpr float kPreCatchForwardDeg = 0.0f;      // 前后电机预备摆位目标绝对角 (占位, 现场校准)
    static constexpr uint32_t kPreCatchTimeoutMs = 30000;   // PRE_CATCH 等待按A的超时 (30秒)

    // ========================================================================
    // 协作对象 (不拥有生命周期)
    // ========================================================================
    ArmControl* arm = nullptr;        // 机械臂电机控制 + 集成视觉对准 (地址 5/6/7)

    ColorQueue color_queue;           // 物料颜色待抓队列 (FIFO; 阶段1 入队, 阶段2 出队)
    PlaceSequence place;              // 阶段2 放置序列 (委托; 共享 color_queue)

    // ========================================================================
    // 运行时状态 (阶段1)
    // ========================================================================
    bool active = false;                    // 阶段1 是否正在运行
    VerticalState state = IDLE;             // 当前状态机状态
    uint32_t grip_start_time = 0;           // 进入夹取等待状态的时间戳
    uint32_t descent_start_time = 0;        // 下降开始时间戳
    uint32_t ascent_start_time = 0;         // 回升/归位开始时间戳
    float vertical_target_degrees = 0.0f;   // 上下电机目标角度 (到位判定)
    float forward_target_degrees = 0.0f;    // 前后电机收臂归位目标角度
    float angle_target_degrees = 0.0f;      // 角度电机收臂归位目标角度
    bool servo_stopped = false;             // 非串流位置控制期是否已停一次伺服
    bool color_recorded_this_run = false;   // 本轮是否已记录过颜色, 防止重复入队

    // 触发边沿检测
    bool prev_a_pressed = false;            // 上一帧 A 键是否按住
    bool prev_x_pressed = false;            // 上一帧 X 键是否按住
    bool prev_l1_pressed = false;           // 上一帧 L1 键是否按住

public:
    // 禁用拷贝和赋值
    ArmSequence(const ArmSequence&) = delete;
    ArmSequence& operator=(const ArmSequence&) = delete;

    /**
     * @brief 构造机械臂任务编排器
     * @param arm 已初始化的机械臂控制对象指针 (含视觉对准)
     */
    explicit ArmSequence(ArmControl* arm)
        : arm(arm), place(arm, &color_queue) {}

    /// @brief 是否有动作正在运行 (阶段1 或阶段2; 供 main 安全判断)
    bool isActive() const { return active || place.isActive(); }

    /// @brief 当前颜色队列中待抓物料数量
    uint8_t queuedColorCount() const { return color_queue.count(); }

    /**
     * @brief 每帧调用的主入口 (阶段1 编排 + 阶段2 委托 + 颜色队列)
     * @param aPressed     A 键: 阶段1, 记录一个颜色入队并跑完整序列
     * @param xPressed     X 键: 阶段2, 委托 PlaceSequence (启动 / 人工确认门)
     * @param abortPressed B 键: 中止当前运行, 复位为 IDLE
     * @param l1Pressed    L1 键: 阶段1 前置, 空闲时角度电机预备摆位 (PRE_CATCH)
     */
    void update(bool aPressed, bool xPressed, bool abortPressed, bool l1Pressed) {
        const uint32_t currentTime = millis();
        bool aRising = aPressed && !prev_a_pressed;
        bool xRising = xPressed && !prev_x_pressed;
        bool l1Rising = l1Pressed && !prev_l1_pressed;

        // ---- 阶段2 运行期: 全权委托 PlaceSequence (阶段1 与阶段2 互斥) ----
        if (place.isActive()) {
            place.update(currentTime, xRising, abortPressed);
            prev_a_pressed = aPressed;
            prev_x_pressed = xPressed;
            prev_l1_pressed = l1Pressed;
            return;
        }

        // ---- 阶段1 中止: 运行期按 B → 立即停止并复位 ----
        if (active && abortPressed) {
            Serial.println("Sequence aborted by user");
            // 回滚: 本轮若已记颜色则丢弃, 并松开夹爪 (无论处于哪个子状态)。
            // 必须在 stop() 之前: stop()→haltAndReset() 不碰队列, 但语义上先回滚。
            if (color_recorded_this_run) {
                color_queue.dropLast();
                Serial.printf("Aborted: dropped recorded color, queue size = %u\n", color_queue.count());
            }
            if (arm != nullptr) {
                arm->release();
            }
            stop();
            prev_a_pressed = aPressed;
            prev_x_pressed = xPressed;
            prev_l1_pressed = l1Pressed;
            return;
        }

        // ---- 启动 (仅空闲时响应上升沿): A → 阶段1, X → 阶段2, L1 → 阶段1预备摆位 ----
        if (!active) {
            if (aRising) {
                startRecordSequence(currentTime);
            } else if (xRising) {
                place.tryStart(currentTime);   // 阶段2: 队空则不启动
            } else if (l1Rising) {
                startPreCatch(currentTime);
            }
        }
        //阶段1: PRE_CATCH 期间按 A → 请求误差1并进入 PRE_ALIGN
        else if (state == PRE_CATCH && aRising) {
            Serial.println("[Stage 1] A-key: leaving pre-catch, entering pre-align");
            arm->beginAlign(1, kColorNone, currentTime);  // 请求误差1 + resetPID + 收敛清零
            enterPreAlign(currentTime);
        }
        // PRE_ALIGN 期间按第二次 A → 仅当已收敛时确认并下降
        else if (state == PRE_ALIGN && aRising) {
            if (arm->isAligned()) {
                confirmAlignAndDescend(currentTime);
            } else {
                Serial.println("[Stage 1] Not aligned yet, A-confirm ignored");
            }
        }

        prev_a_pressed = aPressed;
        prev_x_pressed = xPressed;
        prev_l1_pressed = l1Pressed;

        // ---- 阶段1 自锁运行 ----
        if (active) {
            if (arm->isStreaming()) {
                arm->retryAlignIfNeeded(currentTime);
                runVisualServo(currentTime);
                servo_stopped = false;
            } else if ((state == DESCENDING || state == STOW_LIFT_FORWARD ||
                        state == STOW_ANGLE || state == STOW_DESCEND) && !servo_stopped) {
                // 非串流位置控制期停一次伺服 (排除 PRE_CATCH/PRE_ALIGN)。
                arm->stopServo();
                servo_stopped = true;
            }
            runVerticalSequence(currentTime);
        }
    }

    /**
     * @brief 立即停止所有机械臂电机并复位状态机 (阶段1 + 阶段2)
     * @note  中止 / 紧急停止 / 手柄断连超时时调用。颜色队列保留。
     */
    void stop() {
        place.stop();
        haltAndReset();
    }

    /// @brief 清空颜色队列 (供 main 在需要时手动复位任务)
    void clearColorQueue() {
        color_queue.clear();
    }

private:
    // ========================================================================
    // 停止原语
    // ========================================================================

    /// @brief 停所有臂电机 + 停视觉串流 (不动状态机/自锁)
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

    /// @brief 彻底停止: 停电机 + 复位状态机 (颜色队列不在此清空)
    void haltAndReset() {
        haltMotors();
        resetRunState();
    }

    // ========================================================================
    // 序列启动 (阶段1)
    // ========================================================================

    /**
     * @brief 阶段1启动 (A 键上升沿): 请求误差1, 握手后记录一个颜色, 跑完整升降序列
     * @param currentTime 当前 millis() 时间戳
     */
    void startRecordSequence(uint32_t currentTime) {
        active = true;
        servo_stopped = false;
        color_recorded_this_run = false;
        Serial.println("[Stage 1] Record: requesting error1");
        arm->beginAlign(1, kColorNone, currentTime);  // 请求误差1 + resetPID + 收敛清零
         enterPreAlign(currentTime);
    }

    /**
     * @brief PRE_CATCH 预备摆位启动 (L1 上升沿, 仅空闲): 角度电机转到预备角, 等按 A
     * @param currentTime 当前 millis() 时间戳
     * @note  前置步骤: 自锁但不开视觉、不跑 PID; 只发一次角度位置命令。
     */
    void startPreCatch(uint32_t currentTime) {
        active = true;
        servo_stopped = false;
        color_recorded_this_run = false;
        Serial.printf("[Stage 1] Pre-catch: angle motor to %.1f deg, forward motor to %.1f deg, waiting for A\n",
                      kPreCatchAngleDeg, kPreCatchForwardDeg);
        // 角度+前后打包成一条长指令一次性下发 (同时摆位, 避免背靠背发送丢帧)
        arm->setAngleForwardPosition(kPreCatchAngleDeg, kPreCatchForwardDeg);

        descent_start_time = currentTime;   // 复用作 PRE_CATCH 超时基准
        state = PRE_CATCH;
    }

    /**
     * @brief 进入 PRE_ALIGN: 上下位置先降一小段, 同时跑角度+前后 PID 对准, 等第二次按 A
     * @param currentTime 当前 millis() 时间戳
     * @note  供两处复用: IDLE+A 直接进 (startRecordSequence), 以及 PRE_CATCH+A 转入。
     *        调用前视觉串流应已请求 (arm->beginAlign 已调)。
     */
    void enterPreAlign(uint32_t currentTime) {
        Serial.printf("[Stage 1] Pre-descend %.2f rotations, waiting for A-confirm\n", kPreDescendRotations);
        float preTargetDeg = kPreDescendRotations * 360.0f;  // 符号已在常量里 (正=下降)
        vertical_target_degrees = preTargetDeg;
        arm->setVerticalPosition(preTargetDeg);
        descent_start_time = currentTime;  // 复用作 PRE_ALIGN 超时基准
        state = PRE_ALIGN;
    }

    /**
     * @brief PRE_ALIGN 确认 → 停视觉伺服 + 位置下降到夹取位
     * @param currentTime 当前 millis() 时间戳
     */
    void confirmAlignAndDescend(uint32_t currentTime) {
        Serial.println("[Stage 1] confirm: aligned, descending to grip position");
        arm->stopAlignStream();   // 下降期间停视觉伺服
        arm->stopServo();
        delay(2);
        arm->setCountConvergence(false);
        float targetDeg = kDescendTargetRotations * 360.0f;  // 符号已在常量里
        vertical_target_degrees = targetDeg;
        arm->setVerticalPosition(targetDeg);
        state = DESCENDING;
        descent_start_time = currentTime;
    }

    /**
     * @brief 本轮升降序列正常跑完后的收尾 (阶段1: 一轮记一个颜色, 跑完即结束)
     * @param currentTime 当前 millis() 时间戳
     */
    void finishRun(uint32_t currentTime) {
        haltMotors();  // 停本轮串流与电机
        // 握手失败 → 整轮没记到颜色, 提示用户 (队列未增长)
        if (!color_recorded_this_run) {
            Serial.println("[Stage 1] WARNING: no color recorded this run (no vision data)");
        }
        resetRunState();
    }

    /**
     * @brief 视觉伺服一帧 (委托 arm->updateAlign) + 阶段1 记色入队
     * @param currentTime 当前 millis() 时间戳
     * @note  收敛达标时打印 "[Stage 1] Aligned!" 提示, 等第二次按 A 由 update() 处理。
     */
    void runVisualServo(uint32_t currentTime) {
        ArmControl::AlignFrame f = arm->updateAlign(currentTime);

        // 阶段1: 握手成功后记录本轮第一个颜色入队 (每轮只记一个)
        if (f.freshFrame && !color_recorded_this_run) {
            if (color_queue.enqueue(f.color)) {
                Serial.printf("Recorded color %u, queue size = %u\n", f.color, color_queue.count());
            } else {
                Serial.println("WARNING: Color queue full, dropping color");
            }
            color_recorded_this_run = true;
        }

        // 收敛达标提示 (仅 PRE_ALIGN 期, 提示一次按 A)
        if (state == PRE_ALIGN && f.converged) {
            static uint32_t last_hint = 0;
            if ((currentTime - last_hint) > 1000) {
                Serial.println("[Stage 1] Aligned! Press A to confirm grip.");
                last_hint = currentTime;
            }
        }
    }

    /**
     * @brief 阶段1 上下电机自动序列状态机
     *        PRE_CATCH → PRE_ALIGN → DESCENDING → GRIPPING
     *        → STOW_LIFT_FORWARD → STOW_ANGLE → STOW_DESCEND → IDLE
     * @param currentTime 当前 millis() 时间戳
     */
    void runVerticalSequence(uint32_t currentTime) {
        switch (state) {
            case IDLE:
                break;

            // ---- PRE_CATCH: 角度电机已发位置命令, 等按 A; 此处仅超时防护 ----
            case PRE_CATCH:
                if ((currentTime - descent_start_time) > kPreCatchTimeoutMs) {
                    Serial.println("ERROR: Pre-catch timeout (no A-press)! Stopping.");
                    haltAndReset();
                }
                break;

            // ---- PRE_ALIGN: 等第二次按A (update 处理), 此处超时防护 ----
            case PRE_ALIGN:
                if ((currentTime - descent_start_time) > kPreAlignTimeoutMs) {
                    Serial.println("ERROR: Pre-align timeout (no second A-press)! Stopping.");
                    haltAndReset();
                }
                break;

            // ---- DESCENDING: 位置控制下降到夹取位, 位置反馈轮询到位 ----
            case DESCENDING: {
                float currentPos = 0.0f;
                if (arm->readVerticalPosition(currentPos)) {
                    if (fabsf(currentPos - vertical_target_degrees) <= kPositionArrivedThreshDeg) {
                        Serial.printf("Descent complete (pos=%.1f), gripping\n", currentPos);
                        arm->grip();    // 到位夹紧物料
                        state = GRIPPING;
                        grip_start_time = currentTime;
                    }
                }
                if ((currentTime - descent_start_time) > kDescentTimeoutMs) {
                    Serial.println("ERROR: Descent timeout! Stopping.");
                    haltAndReset();
                }
                break;
            }

            // ---- GRIPPING: 夹取 settle 后进收臂归位步骤1 ----
            case GRIPPING: {
                if ((currentTime - grip_start_time) >= kGripSettleMs) {
                    float vDeg = kStowLiftRotations * 360.0f;
                    vertical_target_degrees = vDeg;
                    forward_target_degrees = kStowForwardDeg;
                    Serial.printf("Grip done (%lums), stow step1: vertical→%.1f, forward→%.1f\n",
                                  (unsigned long)kGripSettleMs, vDeg, kStowForwardDeg);
                    arm->setVerticalPosition(vDeg);
                    arm->setForwardPosition(kStowForwardDeg);
                    state = STOW_LIFT_FORWARD;
                    ascent_start_time = currentTime;
                }
                break;
            }

            // ---- STOW_LIFT_FORWARD (收臂步骤1): 上下+前后同时归位, 两轴都到位才进下一步 ----
            case STOW_LIFT_FORWARD: {
                float vPos = 0.0f, fPos = 0.0f;
                bool vArrived = arm->readVerticalPosition(vPos) &&
                                fabsf(vPos - vertical_target_degrees) <= kPositionArrivedThreshDeg;
                bool fArrived = arm->readForwardPosition(fPos) &&
                                fabsf(fPos - forward_target_degrees) <= kPositionArrivedThreshDeg;
                if (vArrived && fArrived) {
                    Serial.println("Stow step1 done, stow step2: angle homing");
                    angle_target_degrees = kStowAngleDeg;
                    arm->setAnglePosition(kStowAngleDeg);
                    state = STOW_ANGLE;
                    ascent_start_time = currentTime;
                }
                if ((currentTime - ascent_start_time) > kAscentTimeoutMs) {
                    Serial.println("ERROR: Stow step1 timeout! Stopping.");
                    haltAndReset();
                }
                break;
            }

            // ---- STOW_ANGLE (收臂步骤2): 角度电机归位, 到位进下一步 ----
            case STOW_ANGLE: {
                float aPos = 0.0f;
                if (arm->readAnglePosition(aPos)) {
                    if (fabsf(aPos - angle_target_degrees) <= kPositionArrivedThreshDeg) {
                        float vDeg = kFinalDescendRotations * 360.0f;
                        Serial.printf("Stow step2 done, stow step3: vertical descend→%.1f\n", vDeg);
                        vertical_target_degrees = vDeg;
                        arm->setVerticalPosition(vDeg);
                        state = STOW_DESCEND;
                        descent_start_time = currentTime;
                    }
                }
                if ((currentTime - ascent_start_time) > kAscentTimeoutMs) {
                    Serial.println("ERROR: Stow step2 (angle) timeout! Stopping.");
                    haltAndReset();
                }
                break;
            }

            // ---- STOW_DESCEND (收臂步骤3): 上下再降到位 → release 放料 → 结束 ----
            case STOW_DESCEND: {
                float vPos = 0.0f;
                if (arm->readVerticalPosition(vPos)) {
                    if (fabsf(vPos - vertical_target_degrees) <= kPositionArrivedThreshDeg) {
                        Serial.printf("Stow step3 done (pos=%.1f), releasing\n", vPos);
                        arm->release();   // 收臂到位, 松爪放下物料
                        finishRun(currentTime);
                    }
                }
                if ((currentTime - descent_start_time) > kDescentTimeoutMs) {
                    Serial.println("ERROR: Stow step3 (descend) timeout! Stopping.");
                    haltAndReset();
                }
                break;
            }
        }
    }
};

