#pragma once
#include <Arduino.h>
#include <ArmControl.h>
#include <VisionSerial.h>

/**
 * @brief 机械臂高层任务编排: 阶段1 (夹取 + 收臂归位)
 * @note  阶段1 (A键夹取) 编排留在本类。阶段2 (X键放置) 已移到 TaskCoordinator,
 *        本类不再持有 PlaceSequence / ColorQueue, 也不再处理 X 键。
 *        视觉对准复用 ArmControl 集成的对准接口 (beginAlign/updateAlign/...),
 *        本类不再直接持有 VisionSerial/握手。
 * @note  依赖注入: 构造传入已初始化的 ArmControl 指针, 本类不拥有其生命周期。
 * @note  对外接口: main/TaskCoordinator 调 update(a,x,b,l1); x 参数已忽略 (放置移走)。
 * @note  == Version 4.0.0 (color queue & placement removed) ==
 */
class ArmSequence {
private:
    // ========================================================================
    // 上下电机状态机 (阶段1: A键夹取, 收臂归位)
    //   IDLE →[A或L1] PRE_CATCH(角度+前后预备摆位,等A) →[A] PRE_ALIGN(PID对准,等A确认)
    //   →[A] DESCENDING(位置下降到夹取位) → GRIPPING(舵机夹取+settle)
    //   → STOW_LIFT_FORWARD(上下→0 + 前后→0 同时) → STOW_ANGLE(角度→0)
    //   → STOW_FORWARD_EXTEND(前后→-430伸出) → release放料 → [物料盘逻辑链] → IDLE
    // 阶段2 (X键放置) 已移到 TaskCoordinator, 本类只编排阶段1。
    // ========================================================================
    enum VerticalState {
        IDLE,               // 空闲: 机械臂未在执行上下动作
        PRE_CATCH,          // [前置] 预备摆位: 按 A/L1 后角度+前后摆到预备位, 等再按 A
        PRE_ALIGN,          // 对准等待: 角度+前后跑PID对准, 等再按 A 确认
        DESCENDING,         // 下降中: 位置控制到夹取位
        GRIPPING,           // 底部动作: 舵机夹取 + settle
        STOW_LIFT_FORWARD,  // [收臂] 步骤1: 上下→0 + 前后→0 同时, 轮询两轴到位
        STOW_ANGLE,         // [收臂] 步骤2: 角度电机→0, 轮询到位
        STOW_FORWARD_EXTEND,// [收臂] 步骤3: 前后电机伸出到 430, 到位后 release 放料
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
    // 步骤1: 上下→0 + 前后→0 同时; 步骤2: 角度→0; 步骤3: 前后伸出到 kStowForwardExtendDeg
    static constexpr float kStowVerticalHomeDeg = 0.0f;    // 步骤1 上下电机归位绝对角 (0 位)
    static constexpr float kStowForwardHomeDeg = 0.0f;     // 步骤1 前后电机归位绝对角 (0 位)
    static constexpr float kStowAngleDeg = 0.0f;           // 步骤2 角度电机归位绝对角 (0 位)
    static constexpr float kStowForwardExtendDeg = 430.0f;// 步骤3 前后电机伸出绝对角 (放料位)

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
    bool completed_normally = false;        // 本轮是否跑到自然终点 (finishRun); 供入仓握手区分"成功"与"超时/中止"

    // 触发边沿检测
    bool prev_a_pressed = false;            // 上一帧 A 键是否按住
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
        : arm(arm) {}

    /// @brief 阶段1 是否有动作正在运行 (供 main 安全判断)
    bool isActive() const { return active; }

    /// @brief 上一轮阶段1 是否跑到自然终点 (release 放料后 finishRun)
    /// @note  供入仓握手在 active→idle 下降沿区分: true=正常完成可 store(count+1),
    ///        false=被超时/中止/急停强停, 不应记入仓。startPreCatch 启动时清零。
    bool completedNormally() const { return completed_normally; }

    /**
     * @brief 每帧调用的主入口 (阶段1 编排)
     * @param aPressed     A 键: 阶段1 夹取 (对准确认 / 下降夹取)
     * @param xPressed     X 键: 已忽略 (阶段2 放置移到 TaskCoordinator)
     * @param abortPressed B 键: 中止当前运行, 复位为 IDLE
     * @param l1Pressed    L1 键: 阶段1 前置, 空闲时角度电机预备摆位 (PRE_CATCH)
     */
    void update(bool aPressed, bool xPressed, bool abortPressed, bool l1Pressed) {
        (void)xPressed;   // 放置已移走, X 不在本类处理
        const uint32_t currentTime = millis();
        bool aRising = aPressed && !prev_a_pressed;
        bool l1Rising = l1Pressed && !prev_l1_pressed;

        // ---- 阶段1 中止: 运行期按 B → 立即停止并复位 ----
        if (active && abortPressed) {
            Serial.println("Sequence aborted by user");
            if (arm != nullptr) {
                arm->release();   // 中止松开夹爪 (无论处于哪个子状态)
            }
            stop();
            prev_a_pressed = aPressed;
            prev_l1_pressed = l1Pressed;
            return;
        }

        // ---- 启动 (仅空闲时响应上升沿): A 或 L1 → 角度+前后摆到预备位 (PRE_CATCH) ----
        if (!active) {
            if (aRising || l1Rising) {
                startPreCatch(currentTime);
            }
        }
        //阶段1: PRE_CATCH 期间按 A (第2次) → 请求误差1并进入 PRE_ALIGN
        else if (state == PRE_CATCH && aRising) {
            Serial.println("[Stage 1] A-key: leaving pre-catch, entering pre-align");
            arm->beginAlign(1, kColorNone, currentTime);  // 请求误差1 + resetPID + 收敛清零
            enterPreAlign(currentTime);
        }
        // PRE_ALIGN 期间按 A (第3次) → 仅当已收敛时确认并下降
        else if (state == PRE_ALIGN && aRising) {
            if (arm->isAligned()) {
                confirmAlignAndDescend(currentTime);
            } else {
                Serial.println("[Stage 1] Not aligned yet, A-confirm ignored");
            }
        }

        prev_a_pressed = aPressed;
        prev_l1_pressed = l1Pressed;

        // ---- 阶段1 自锁运行 ----
        if (active) {
            if (arm->isStreaming()) {
                arm->retryAlignIfNeeded(currentTime);
                runVisualServo(currentTime);
                servo_stopped = false;
            } else if ((state == DESCENDING || state == STOW_LIFT_FORWARD ||
                        state == STOW_ANGLE || state == STOW_FORWARD_EXTEND) && !servo_stopped) {
                // 非串流位置控制期停一次伺服 (排除 PRE_CATCH/PRE_ALIGN)。
                arm->stopServo();
                servo_stopped = true;
            }
            runVerticalSequence(currentTime);
        }
    }

    /**
     * @brief 立即停止所有机械臂电机并复位状态机 (阶段1)
     * @note  中止 / 紧急停止 / 手柄断连超时时调用。
     */
    void stop() {
        haltAndReset();
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
     * @brief PRE_CATCH 预备摆位启动 (A 或 L1 上升沿, 仅空闲): 角度+前后摆到预备位, 等再按 A
     * @param currentTime 当前 millis() 时间戳
     * @note  前置步骤: 自锁但不开视觉、不跑 PID; 角度+前后打包一次发出。
     *        之后按第 2 次 A → 请求误差对准 (PRE_ALIGN), 第 3 次 A → 确认下降夹取。
     */
    void startPreCatch(uint32_t currentTime) {
        active = true;
        servo_stopped = false;
        completed_normally = false;   // 新一轮开始: 清成功标志, 只有跑到 finishRun 才置回 true
        Serial.printf("[Stage 1] Pre-catch: open gripper, angle motor to %.1f deg, forward motor to %.1f deg, waiting for A\n",
                      kPreCatchAngleDeg, kPreCatchForwardDeg);
        arm->release();   // 预备位先张开夹爪 (3号舵机), 准备夹取
        delay(2);         // 舵机命令与总线电机命令间留间隔
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
        // [ ! ] 本帧已亲自停过伺服, 标记 servo_stopped 以免 update() 的 active 块紧接着
        //       又发一条 stopServo(), 那会 0 间隔贴在刚发的下降命令(0xFD)后把它挤掉/错帧
        //       → 6号电机收不到下降命令而不动, 进而 DESCENDING 永不到位、熬到超时。
        servo_stopped = true;
        delay(2);   // 与本帧随后 DESCENDING 首次轮询的 ask_current_position 隔开, 同理防丢命令
    }

    /**
     * @brief 本轮升降序列正常跑完后的收尾 (阶段1 跑完即结束)
     * @param currentTime 当前 millis() 时间戳
     */
    void finishRun(uint32_t currentTime) {
        (void)currentTime;
        completed_normally = true;   // 跑到自然终点: 供入仓握手判定"正常完成", 可 store(count+1)
        haltMotors();  // 停本轮串流与电机
        resetRunState();
    }

    /**
     * @brief 视觉伺服一帧 (委托 arm->updateAlign)
     * @param currentTime 当前 millis() 时间戳
     * @note  收敛达标时打印 "[Stage 1] Aligned!" 提示, 等第二次按 A 由 update() 处理。
     */
    void runVisualServo(uint32_t currentTime) {
        ArmControl::AlignFrame f = arm->updateAlign(currentTime);

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
     *        → STOW_LIFT_FORWARD → STOW_ANGLE → STOW_FORWARD_EXTEND → release → IDLE
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

            // ---- GRIPPING: 夹取 settle 后进收臂归位步骤1 (上下→0 + 前后→0) ----
            case GRIPPING: {
                if ((currentTime - grip_start_time) >= kGripSettleMs) {
                    vertical_target_degrees = kStowVerticalHomeDeg;
                    forward_target_degrees = kStowForwardHomeDeg;
                    Serial.printf("Grip done (%lums), stow step1: vertical→%.1f, forward→%.1f\n",
                                  (unsigned long)kGripSettleMs, kStowVerticalHomeDeg, kStowForwardHomeDeg);
                    arm->setVerticalForwardPosition(kStowVerticalHomeDeg, kStowForwardHomeDeg);  // 上下+前后打包一次发出
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

            // ---- STOW_ANGLE (收臂步骤2): 角度电机→0, 到位后进步骤3 (前后伸出) ----
            case STOW_ANGLE: {
                float aPos = 0.0f;
                if (arm->readAnglePosition(aPos)) {
                    if (fabsf(aPos - angle_target_degrees) <= kPositionArrivedThreshDeg) {
                        Serial.printf("Stow step2 done, stow step3: forward extend→%.1f\n", kStowForwardExtendDeg);
                        forward_target_degrees = kStowForwardExtendDeg;
                        arm->setForwardPosition(kStowForwardExtendDeg);
                        state = STOW_FORWARD_EXTEND;
                        descent_start_time = currentTime;
                    }
                }
                if ((currentTime - ascent_start_time) > kAscentTimeoutMs) {
                    Serial.println("ERROR: Stow step2 (angle) timeout! Stopping.");
                    haltAndReset();
                }
                break;
            }

            // ---- STOW_FORWARD_EXTEND (收臂步骤3): 前后伸出到位 → release 放料 → [物料盘逻辑链] → 结束 ----
            case STOW_FORWARD_EXTEND: {
                float fPos = 0.0f;
                if (arm->readForwardPosition(fPos)) {
                    if (fabsf(fPos - forward_target_degrees) <= kPositionArrivedThreshDeg) {
                        Serial.printf("Stow step3 done (pos=%.1f), releasing\n", fPos);
                        arm->release();   // 前后伸出到位, 松爪放下物料
                        // [ ! ] 预留: 物料盘逻辑链入口 (放料后物料盘存储)。
                        //   当前直接 finishRun 结束本轮; 物料盘动作就位后在此衔接 tray->store(color)。
                        finishRun(currentTime);
                    }
                }
                if ((currentTime - descent_start_time) > kDescentTimeoutMs) {
                    Serial.println("ERROR: Stow step3 (forward extend) timeout! Stopping.");
                    haltAndReset();
                }
                break;
            }
        }
    }
};

