#pragma once
#include <Arduino.h>
#include <ArmControl.h>
#include <ArmSequence.h>
#include <TrayControl.h>

/**
 * @brief 上层任务协调器: 阶段1(夹取入仓) 编排 + 阶段2(放置出料) 流程状态机
 * @note  按"流程在 TaskCoordinator、其他类只提供动作"的架构: arm/tray 只提供动作原语,
 *        放置流程的步骤编排全部放在本类 (取代原 PlaceSequence)。
 * @note  阶段2 放置流程为纯手动逐步确认: 每按一次 X 推进一步 (人确认上一步到位),
 *        不做视觉对准、不做位置轮询自动判定。瞄准阶段用手柄方向键 (d-pad) 位置点动。
 * @note  依赖注入: 构造传入 ArmControl / ArmSequence / TrayControl 指针, 本类不拥有其生命周期。
 *        持有 ArmControl 是因为放置流程要直接驱动臂电机 (阶段1 仍由 ArmSequence 编排)。
 * @note  阶段1 入仓握手: 阶段1 启动时 prepareStore(), 正常结束时 store() (count+1),
 *        中止时 tray->stop() 取消入仓。count 是阶段2 dispense() 出料的前提。
 * @note  阶段1 与阶段2 互斥运行 (同一 ArmControl, 二选一)。
 * @note  [ ! ] 所有位置/角度为占位常量, 逻辑未实车验证, 首跑前需现场校准 (撞机/夹空风险)。
 * @note  [ ! ] 共享 Serial2 总线: 同帧连发两条电机命令之间须留 ~2ms, 否则丢命令/读不到回复。
 * @note  == Version 1.0.0 (placement flow moved in; vision removed) ==
 */
class TaskCoordinator {
private:
    // ========================================================================
    // 阶段2 放置流程状态机 (X 键逐步推进)
    //   P_IDLE →(X,count>0) P_TO_PORT →(X) P_DISPENSE →(X) P_GRIP →(X) P_FWD_NEXT
    //   →(X) P_ROTATE →(X) P_DESCEND_AIM →(X) P_AIM[手动d-pad] →(X) P_DESCEND_BOTTOM
    //   →(X) P_RELEASE →(X) [回位: P_RET_LIFT →到位 P_RET_ANGLE →到位 P_RET_EXTEND →到位]
    //   →(count>0) P_DISPENSE / (count==0) P_IDLE
    // 注: 回位 P_RET_* 是放置流程唯一带自动到位判定的段 (撞机风险高, 须按序到位; 同阶段1 STOW)。
    // ========================================================================
    enum PlaceState {
        P_IDLE,            // 空闲
        P_TO_PORT,         // 开爪 + 转到出料口 (首次进入额外 compact 压紧)
        P_DISPENSE,        // 物料盘顶料舵机顶起一个物块
        P_GRIP,            // 夹爪合上, 领取物块
        P_FWD_NEXT,        // r轴(前后)前伸到头 + 物料盘 notifyPicked (落下/送下一块/count-1)
        P_ROTATE,          // 角度转180° (先转完, 不动上下)
        P_DESCEND_AIM,     // 上下下降一段 (转完后再降, 进手动瞄准前置)
        P_AIM,             // 手动模式: d-pad 点动瞄准靶子; 按 X 推进
        P_DESCEND_BOTTOM,  // 上下下降到底
        P_RELEASE,         // 夹爪张开, 放置完成
        // ---- 回位 (按一次 X 进入, 内部自动依次到位; 防撞顺序: 先收上下+前后→0, 再角度→0, 最后前后伸出) ----
        P_RET_LIFT,        // 回位step1: 上下→0 + 前后→0 (打包), 轮询两轴到位
        P_RET_ANGLE,       // 回位step2: 角度→0, 轮询到位
        P_RET_EXTEND,      // 回位step3: 前后→+430 领料位, 到位后 续下一块/结束
    };

    // ========================================================================
    // 放置流程占位参数 (全部 [ ! ] 待现场校准; 方向反了改符号/数值)
    // ========================================================================
    static constexpr float kPortAngleDeg     = 0.0f;     // 出料口: 角度电机绝对角 (占位)
    static constexpr float kForwardEndDeg     = 0.0f;     // r轴(前后)前伸到头: 绝对角 (占位)
    static constexpr float kRotate180Deg      = -180.0f;   // 转180°: 角度电机绝对角 (占位)
    static constexpr float kAimDescendDeg     = -5100.0f;   // 瞄准前下降一段: 上下绝对角 (占位)
    static constexpr float kBottomDeg         = -5300.0f;   //降到底: 上下绝对角 (占位)
    static constexpr float kReturnVerticalDeg      = 0.0f;     // 回位 step1: 上下→0
    static constexpr float kReturnForwardHomeDeg   = 0.0f;     // 回位 step1: 前后→0 (先收回, 防撞)
    static constexpr float kReturnAngleDeg         = 0.0f;     // 回位 step2: 角度→0
    static constexpr float kReturnForwardExtendDeg = 420.0f;   // 回位 step3: 前后伸到领料位 (+430, 与夹取收臂 kStowForwardExtendDeg 方向统一)

    // 回位到位判定 (P_RETURN_* 自动轮询, 撞机风险高需按序保证)
    static constexpr float kPositionArrivedThreshDeg = 5.0f;   // 到位阈值 (度)
    static constexpr uint32_t kReturnTimeoutMs       = 12000;  // 回位每段超时保护 (ms)

    // 手动点动 (d-pad 位置步进)
    static constexpr float kJogStepDeg    = 2.0f;    // 每帧按住步进量 (度, 占位)
    static constexpr float kJogAngleSign  = -0.2f; // 左右→角度 方向因子 (反了取负)
    static constexpr float kJogForwardSign = -1.0f;   // 上下→前后 方向因子 (反了取负)

    static constexpr uint32_t kStepLockMs = 200;     // 步进后忽略 X 的最短停留 (防手快误推进)
    static constexpr uint32_t kBusDelayMs = 2;       // 同帧两条命令间的总线间隔

    // ========================================================================
    // 协作对象 (不拥有生命周期)
    // ========================================================================
    ArmControl* arm = nullptr;        // 机械臂电机控制 (放置流程直接驱动)
    ArmSequence* arm_seq = nullptr;   // 阶段1 夹取序列 (编排留在其内)
    TrayControl* tray = nullptr;      // 物料盘 (存料/出料动作)

    // ========================================================================
    // 运行时状态
    // ========================================================================
    PlaceState place_state = P_IDLE;  // 放置流程当前状态
    bool compacted = false;           // 本轮放置是否已压紧 (compact 只首次做一次)
    uint32_t step_time = 0;           // 上次步进时间戳 (kStepLockMs 基准)

    // 手动点动目标 (跟踪下发值, 不读位置)
    float angle_target = 0.0f;        // 角度电机当前目标角
    float forward_target = 0.0f;      // 前后电机当前目标角

    // 阶段1 入仓握手: 跟踪 arm_seq 忙闲边沿
    bool prev_arm_seq_active = false; // 上一帧阶段1 是否在运行

    // X 键边沿检测 (放置流程自持, 与 ArmSequence 的边沿独立)
    bool prev_x_pressed = false;

    // Y 键边沿检测 (手动物料计数 +1, 全局生效)
    bool prev_y_pressed = false;

public:
    // 禁用拷贝和赋值
    TaskCoordinator(const TaskCoordinator&) = delete;
    TaskCoordinator& operator=(const TaskCoordinator&) = delete;

    /**
     * @brief 构造任务协调器
     * @param arm    机械臂控制 (放置流程直接驱动臂电机)
     * @param armSeq 阶段1 夹取序列 (编排留在其内)
     * @param tray   物料盘控制 (存料/出料)
     */
    TaskCoordinator(ArmControl* arm, ArmSequence* armSeq, TrayControl* tray)
        : arm(arm), arm_seq(armSeq), tray(tray) {}

    /**
     * @brief 每帧主入口 (阶段1 转发 + 阶段2 放置流程 + 物料盘推进 + Y键计数)
     * @param aPressed     A 键: 阶段1 夹取
     * @param xPressed     X 键: 阶段2 放置 (启动 / 逐步确认推进)
     * @param abortPressed B 键: 中止当前运行
     * @param l1Pressed    L1 键: 阶段1 前置预备摆位
     * @param dpadX        方向键左右 (-1/0/1): 手动瞄准角度电机
     * @param dpadY        方向键上下 (-1/0/1): 手动瞄准前后电机
     * @param yPressed     Y 键: 手动物料数 +1 (全局生效, 任意时刻可按)
     * @note  阶段1 与阶段2 互斥: 放置流程运行期独占 X/B/dpad, 不转发 arm_seq。
     * @note  Y 键计数全局独立: 不受阶段互斥影响, 任何状态下按一下都 +1。
     */
    void update(bool aPressed, bool xPressed, bool abortPressed,
                bool l1Pressed, int dpadX, int dpadY, bool yPressed) {
        const uint32_t now = millis();
        const bool xRising = xPressed && !prev_x_pressed;

        // ---- Y 键手动物料计数 +1 (全局生效, 与阶段互斥无关) ----
        if (tray != nullptr && yPressed && !prev_y_pressed) {
            tray->addMaterial();
        }
        prev_y_pressed = yPressed;

        // ---- 阶段2 运行期: 独占放置流程 (与阶段1 互斥) ----
        if (place_state != P_IDLE) {
            if (abortPressed) {
                abortPlacement();
            } else {
                runPlacement(now, xRising, dpadX, dpadY);
            }
        } else {
            // ---- 阶段1 转发 (X 不传给阶段1, 放置已移走) ----
            if (arm_seq != nullptr) {
                arm_seq->update(aPressed, false, abortPressed, l1Pressed);
            }
            delay(kBusDelayMs);
            // 阶段1 入仓握手: 监测 arm_seq 忙闲边沿
            handleStoreHandshake(abortPressed);
            delay(kBusDelayMs); // 同帧两段逻辑间隔 (防总线冲突)
            // 空闲且按 X 且仓非空 → 启动放置流程
            if (xRising && !isArmSeqActive() && tray != nullptr && tray->count() > 0) {
                startPlacement(now);
            } else if (xRising && tray != nullptr && tray->count() <= 0) {
                Serial.println("[Place] tray empty, nothing to dispense");
            }
        }

        // ---- 物料盘状态机每帧推进 (非阻塞) ----
        // [ ! ] 必须用当前 millis(), 不能复用帧开头的 now: 本帧 prepareStore() 用更晚的
        //       millis() 盖了 step_start_time, 若传旧 now 则 (now - step_start_time) 无符号
        //       下溢成巨值 → 第一帧即误判超时 → 刚发的带子命令被 stop() 当帧刹停。
        if (tray != nullptr) {
            tray->update(millis());
        }

        prev_x_pressed = xPressed;
    }

    /// @brief 是否有动作正在运行 (阶段1 / 放置流程 / 物料盘忙; 供 main 安全判断)
    bool isActive() const {
        return isArmSeqActive() ||
               (tray != nullptr && tray->isBusy()) ||
               place_state != P_IDLE;
    }

    /// @brief 立即停止 (中止/急停/手柄断连时调用)
    void stop() {
        if (arm_seq != nullptr) {
            arm_seq->stop();
        }
        if (tray != nullptr) {
            tray->stop();
        }
        place_state = P_IDLE;
        compacted = false;
        prev_arm_seq_active = false;
    }

private:
    /// @brief 阶段1 是否在运行 (空指针安全)
    bool isArmSeqActive() const {
        return arm_seq != nullptr && arm_seq->isActive();
    }

    // ========================================================================
    // 阶段1 入仓握手 (靠 arm_seq 忙闲边沿驱动物料盘存料)
    //   启动(idle→active): prepareStore() 顶料升起+挡片退位 (在阶段1 数秒内 retract)
    //   结束(active→idle, 非中止): store() 落下+推进+count+1
    //   中止(active 期按 B): tray->stop() 取消入仓, count 不增
    // ========================================================================
    void handleStoreHandshake(bool abortPressed) {
        if (tray == nullptr) {
            return;
        }
        const bool active = isArmSeqActive();

        // 上升沿: 阶段1 刚启动 → 准备承接 (顶料升起 + 挡片退到入口位)
        if (active && !prev_arm_seq_active) {
            Serial.println("[Store] stage1 started, prepareStore()");
            delay(kBusDelayMs);   // 与本帧刚发出的 PRE_CATCH 角度/前后命令隔开, 防总线丢命令
            tray->prepareStore();
        }
        // 下降沿: 阶段1 刚结束
        else if (!active && prev_arm_seq_active) {
            // [ ! ] 只看 abortPressed 不够: 下降/收臂超时、急停等也会让 arm_seq 从 active 落回
            //       idle, 但那不是"正常完成", 不应记入仓。改以 arm_seq 是否跑到自然终点
            //       (release 放料后 finishRun) 为准。abort 当帧 completedNormally 仍为 false。
            const bool ok = arm_seq != nullptr && arm_seq->completedNormally();
            if (ok && !abortPressed) {
                // 正常结束: 落下 + 推进 + count+1
                Serial.println("[Store] stage1 done, store() count+1");
                tray->store();
            } else {
                // 超时/出错/中止收尾: 取消本次入仓 (count 不增)
                Serial.println("[Store] stage1 ended without normal completion, cancel store");
                tray->stop();
            }
        }

        prev_arm_seq_active = active;
    }

    // ========================================================================
    // 阶段2 放置流程
    // ========================================================================

    /// @brief 启动放置流程: 进 P_TO_PORT (X 已确认 count>0)
    void startPlacement(uint32_t now) {
        Serial.printf("[Place] start, tray count=%d\n", tray->count());
        compacted = false;
        enterToPort(now);
    }

    /// @brief 进 P_TO_PORT: 开爪 + 转出料口; 首次额外 compact 压紧
    void enterToPort(uint32_t now) {
        arm->release();                       // 开爪 (舵机)
        delay(kBusDelayMs);                   // 总线间隔
        arm->setAnglePosition(kPortAngleDeg); // 转到出料口
        angle_target = kPortAngleDeg;
        if (!compacted && tray != nullptr) {
            delay(kBusDelayMs);
            tray->compact();                  // 出仓前压紧 (只首次)
            compacted = true;
        }
        place_state = P_TO_PORT;
        step_time = now;
        Serial.println("[Place] P_TO_PORT: open gripper, rotate to port");
    }

    /// @brief 中止放置: 松爪 + 停臂 + 停盘 + 复位
    void abortPlacement() {
        Serial.println("[Place] aborted by user");
        arm->release();
        delay(kBusDelayMs);
        arm->stop();
        if (tray != nullptr) {
            tray->stop();
        }
        place_state = P_IDLE;
        compacted = false;
    }

    /**
     * @brief 放置流程每帧推进
     * @param now    当前 millis()
     * @param xRising X 上升沿 (推进一步)
     * @param dpadX/dpadY  手动瞄准方向键 (仅 P_AIM 使用)
     */
    void runPlacement(uint32_t now, bool xRising, int dpadX, int dpadY) {
        // 手动瞄准: 每帧按住即位置步进 (单次总线事务下发角度+前后)
        if (place_state == P_AIM && (dpadX != 0 || dpadY != 0)) {
            if (dpadX != 0) angle_target   += dpadX * kJogStepDeg * kJogAngleSign;
            if (dpadY != 0) forward_target += dpadY * kJogStepDeg * kJogForwardSign;
            arm->setAngleForwardPosition(angle_target, forward_target);
        }

        // 回位段: 每帧自动轮询到位推进 (不等 X; 撞机风险高需按序保证)
        if (place_state == P_RET_LIFT || place_state == P_RET_ANGLE ||
            place_state == P_RET_EXTEND) {
            runReturn(now);
            return;
        }

        if (!xRising) {
            return;
        }
        // 步进锁: 防手快 (上一步动作未完成就推进)
        if ((now - step_time) < kStepLockMs) {
            return;
        }
        advancePlacement(now);
    }

    /**
     * @brief 回位段自动推进 (P_RELEASE 后按一次 X 进入; 内部依次到位)
     * @param now 当前 millis()
     * @note  防撞顺序: 上下+前后→0 (一起) → 角度→0 → 前后伸到 +430 领料位。
     *        每段用 readPosition 轮询到位 (阈值 kPositionArrivedThreshDeg), 带超时保护。
     */
    void runReturn(uint32_t now) {
        switch (place_state) {
            // step1: 上下→0 + 前后→0 都到位 → 转角度→0
            case P_RET_LIFT:
                if (verticalArrived() && forwardArrived()) {
                    arm->setAnglePosition(kReturnAngleDeg);
                    angle_target = kReturnAngleDeg;
                    place_state = P_RET_ANGLE;
                    step_time = now;
                    Serial.println("[Place] P_RET_ANGLE: angle -> 0");
                } else if ((now - step_time) > kReturnTimeoutMs) {
                    Serial.println("ERROR: [Place] return step1 timeout! Aborting.");
                    abortPlacement();
                }
                break;

            // step2: 角度→0 到位 → 前后伸到 +430 领料位
            case P_RET_ANGLE:
                if (angleArrived()) {
                    arm->setForwardPosition(kReturnForwardExtendDeg);
                    forward_target = kReturnForwardExtendDeg;
                    place_state = P_RET_EXTEND;
                    step_time = now;
                    Serial.println("[Place] P_RET_EXTEND: forward -> +430 (pickup pose)");
                } else if ((now - step_time) > kReturnTimeoutMs) {
                    Serial.println("ERROR: [Place] return step2 (angle) timeout! Aborting.");
                    abortPlacement();
                }
                break;

            // step3: 前后到领料位 → 仓非空续下一块, 空则结束
            case P_RET_EXTEND:
                if (forwardArrived()) {
                    if (tray != nullptr && tray->count() > 0) {
                        tray->dispense();   // 下一块已在出料口, 直接顶料 (不再 compact)
                        place_state = P_DISPENSE;
                        step_time = now;
                        Serial.printf("[Place] next block, count=%d\n", tray->count());
                    } else {
                        place_state = P_IDLE;
                        compacted = false;
                        Serial.println("[Place] all done, tray empty");
                    }
                } else if ((now - step_time) > kReturnTimeoutMs) {
                    Serial.println("ERROR: [Place] return step3 (extend) timeout! Aborting.");
                    abortPlacement();
                }
                break;

            default:
                break;
        }
    }

    /// @brief 上下电机到位判定 (读位置且在阈值内)
    bool verticalArrived() {
        float p = 0.0f;
        return arm->readVerticalPosition(p) &&
               fabsf(p - kReturnVerticalDeg) <= kPositionArrivedThreshDeg;
    }
    /// @brief 前后电机到位判定 (按当前 forward_target)
    bool forwardArrived() {
        float p = 0.0f;
        return arm->readForwardPosition(p) &&
               fabsf(p - forward_target) <= kPositionArrivedThreshDeg;
    }
    /// @brief 角度电机到位判定 (按当前 angle_target)
    bool angleArrived() {
        float p = 0.0f;
        return arm->readAnglePosition(p) &&
               fabsf(p - angle_target) <= kPositionArrivedThreshDeg;
    }

    /// @brief X 上升沿推进一步 (人确认上一步到位)
    void advancePlacement(uint32_t now) {
        step_time = now;
        switch (place_state) {
            // P_TO_PORT 已到出料口 → 顶料出一块
            case P_TO_PORT:
                if (tray != nullptr) {
                    tray->dispense();   // 顶料舵机顶起一个物块 (要求 count>0)
                }
                place_state = P_DISPENSE;
                Serial.println("[Place] P_DISPENSE: tray lift one block");
                break;

            // P_DISPENSE 物块已顶起 → 合爪领取
            case P_DISPENSE:
                arm->grip();
                place_state = P_GRIP;
                Serial.println("[Place] P_GRIP: close gripper");
                break;

            // P_GRIP 已夹住 → r轴前伸到头 + 通知物料盘送下一块
            case P_GRIP:
                arm->setForwardPosition(kForwardEndDeg);
                forward_target = kForwardEndDeg;
                delay(kBusDelayMs);
                if (tray != nullptr) {
                    tray->notifyPicked();   // 顶料落下 + 带子送下一块 + count-1
                }
                place_state = P_FWD_NEXT;
                Serial.println("[Place] P_FWD_NEXT: r-axis forward + tray next");
                break;

            // P_FWD_NEXT → 角度转180° (先转完, 不动上下)
            case P_FWD_NEXT:
                arm->setAnglePosition(kRotate180Deg);
                angle_target = kRotate180Deg;
                place_state = P_ROTATE;
                Serial.println("[Place] P_ROTATE: rotate 180");
                break;

            // P_ROTATE 已转完 → 上下下降一段 (进手动瞄准前置)
            case P_ROTATE:
                arm->setVerticalPosition(kAimDescendDeg);
                place_state = P_DESCEND_AIM;
                Serial.println("[Place] P_DESCEND_AIM: descend before aim");
                break;

            // P_DESCEND_AIM → 进手动瞄准模式
            case P_DESCEND_AIM:
                place_state = P_AIM;
                Serial.println("[Place] P_AIM: manual d-pad aim (left/right=angle, up/down=forward), X to confirm");
                break;

            // P_AIM 瞄准完成 → 下降到底
            case P_AIM:
                arm->setVerticalPosition(kBottomDeg);
                place_state = P_DESCEND_BOTTOM;
                Serial.println("[Place] P_DESCEND_BOTTOM: descend to bottom");
                break;

            // P_DESCEND_BOTTOM 已到底 → 松爪放置
            case P_DESCEND_BOTTOM:
                arm->release();
                place_state = P_RELEASE;
                Serial.println("[Place] P_RELEASE: open gripper, placed");
                break;

            // P_RELEASE → 回位 step1: 上下→0 + 前后→0 (打包发出), 之后自动轮询到位
            case P_RELEASE:
                arm->setVerticalForwardPosition(kReturnVerticalDeg, kReturnForwardHomeDeg);
                forward_target = kReturnForwardHomeDeg;
                delay(kBusDelayMs);   // 与下一帧 runReturn 的 readPosition 隔开, 防 0 间隔挤掉刚发的位置命令 (同夹取下降 bug)
                place_state = P_RET_LIFT;
                Serial.println("[Place] P_RET_LIFT: vertical+forward -> 0 (auto arrive)");
                break;

            // 回位段 P_RET_* 由 runReturn() 自动推进, 不在此处理 X
            case P_RET_LIFT:
            case P_RET_ANGLE:
            case P_RET_EXTEND:
                break;

            case P_IDLE:
                break;
        }
    }
};
