#pragma once
#include <ControlSerial.h>
#include <Arduino.h>
#include <cmath>

/**
 * @brief 物料盘控制类 (环形同步带 + 两个舵机)
 * @note  机构: 一条环形同步带 (上贴一个挡片) 把松散排成一列的物料绕圈推送,
 *        出入口共用; 出入口正下方一个顶料舵机, 仓内一侧一个挡板舵机。
 * @note  硬件:
 *          - 同步带步进电机: Serial2 总线地址 8, 绝对角度位置控制
 *            (正值 = 挡片逆时针转 = 让出出入口方向; 负值 = 顺时针 = 把料往仓里推/压紧方向)
 *          - 舵机1 (顶料): LEDC PWM, 两固定位 升起/落下, 初始 = 落下
 *          - 舵机2 (挡板): LEDC PWM, 两固定位 升起/降下, 初始 = 升起
 * @note  与底盘(1~4)、机械臂(5~7)共用同一 Serial2 总线 (ControlSerial 单例)。
 *        舵机 LEDC 通道避开夹爪占用的通道 4。
 * @note  动作以非阻塞状态机实现: store/compact/dispense 仅发起, 每帧 update() 推进,
 *        不使用 delay() 死等, 避免阻塞 50Hz 主循环与急停。
 * @note  本类不维护颜色队列 (颜色由机械臂侧自行读写 ColorQueue); 只用一个整数计数器
 *        material_count 记录仓内物料数量, 作为上层查表的输入。
 * @note  "数量 → 同步带绝对角度" 的对照表属于上层 (TaskCoordinator), 不写进本类;
 *        compact/dispense 的目标角度由调用方按数量查表后传入。
 * @note  == Version 0.2.0 (skeleton → 状态机骨架) ==
 */
class TrayControl {
private:
    // ========================================================================
    // 硬件配置 (占位默认值, 按实际硬件修改)
    // ========================================================================

    // ---- 同步带步进电机 (Serial2 总线) ----
    static constexpr int kBeltAddr = 8;          // 同步带电机总线地址 (避开 1~7)
    static constexpr float kBeltSpeed = 50.0f;   // 同步带移动转速 (rpm), 待调

    // ---- 顶料舵机 (舵机1, LEDC PWM) ----
    static constexpr int kLiftPin = 22;          // 顶料舵机信号引脚 (GPIO22)
    static constexpr int kLiftChannel = 5;       // LEDC 通道 (避开夹爪的 4)
    static constexpr float kLiftUpStoreAngle = 170.0f;     // 进料时顶起角度 (度): 承接机械臂放料
    static constexpr float kLiftUpDispenseAngle = 5.0f; // 出料时顶起角度 (度): 托高物料供机械臂夹取
    static constexpr float kLiftDownAngle = 180.0f;  // 落下角度 (度): 初始位 (上电初始化到此)

    // ---- 挡板舵机 (舵机2, LEDC PWM) ----
    static constexpr int kBafflePin = 23;        // 挡板舵机信号引脚 (GPIO23)
    static constexpr int kBaffleChannel = 6;     // LEDC 通道 (避开夹爪的 4 与顶料的 5)
    static constexpr float kBaffleUpAngle = 87.0f;   // 升起角度 (度): 初始位, 0=下, 90=上
    static constexpr float kBaffleDownAngle = 50.0f;  // 降下角度 (度): 挡住定位

    // ---- 舵机 PWM 公共参数 (同 ArmControl 夹爪) ----
    static constexpr int kPwmFreq = 50;          // 舵机标准 50Hz
    static constexpr int kPwmResBits = 16;       // PWM 分辨率位数
    static constexpr float kMinPulseUs = 500.0f;   // 0 度对应脉宽 (us)
    static constexpr float kMaxPulseUs = 2500.0f;  // 180 度对应脉宽 (us)

    // ---- 入仓动作角度 (绝对角度, 占位, 待测) ----
    // 零点约定: 上电零点 = 物料被推入仓后停的位置 (推入位)。
    //   等待放料时挡片退到正角度 (让出出入口); 放料后推回零点把料送进仓。
    //   方向: 等待位(正, 逆时针让出出入口) → 推入位(0), 挡片顺时针(向负)走把料送入;
    //         压紧/出料继续往负方向(顺时针)推。
    static constexpr float kEntryAngleDeg = 270.0f;    // 等待放料时退回的正角度 (让出入口), 待测
    static constexpr float kStoreInDeg = 0.0f;        // 放料后把物料推入仓的目标角 (=零点), 待测
    static constexpr float kCompactPreDeg = -15.0f;   // 压紧前置位: 先转到此角再降挡板 (零点-30), 待测

    // ---- 压紧 / 出料 角度对照表: 索引 = 仓内物料数量, 值 = 步进电机绝对角度 (度) ----
    // [ ! ] 占位全 0, 必须现场逐个测量。值应为负 (压紧/出料方向为负)。
    //   compact():  用 kTightenTable[count]   把当前 count 个物料压紧。
    //   dispense(): 用 kTightenTable[count-1], 出一个后剩余物料的压紧位 (头块送到出料口)。
    static constexpr float kTightenTable[10] = {
        0.0f,   // 0 个 (空仓 / home)
        -835.0f,   // 1 个
        -717.0f,   // 2 个
        -655.0f,   // 3 个
        -590.0f,   // 4 个
        -456.0f,   // 5 个
        -280.0f,   // 6 个
        -172.0f,   // 7 个
        -85.0f,   // 8 个
        -20.0f,   // 9 个 (最多)
    };

    static constexpr float kPosArrivedThreshDeg = 5.0f;    // 同步带到位判定阈值 (度, 仅 readBeltAngle 备用)
    static constexpr uint32_t kBeltMoveMs = 1000;          // 同步带移动固定等待 (ms): 发命令后等够这段即视为到位 (闭环步进自走到, 不读位置)
    static constexpr uint32_t kServoSettleMs = 500;        // 舵机动作 settle 等待 (ms), 待调
    static constexpr uint32_t kPositionReadDelayMs = 2;    // 读位置前等电机回复 (ms)

    // ========================================================================
    // 状态机
    // ========================================================================
    enum State {
        IDLE,                // 空闲

        // ---- 入仓 (分两次调用: prepareStore 放料前, store 放料后) ----
        STORE_BELT_TO_ENTRY, // prepareStore 后: 同步带先走到接料位 (kEntryAngleDeg), 轮询到位 → 升舵机1
        STORE_RETRACT,       // 步进到位后: 舵机1 升起承接, 等 settle → 等放料
        STORE_WAIT_PLACE,    // 舵机1 已升起, 停此等机械臂放料 + 等 store()
        STORE_LIFT_DOWN,     // store 触发: 舵机1 落下, 等 settle
        STORE_PUSH,          // 同步带挡片推回零点把料送进仓, 轮询到位 → 计数+1 → IDLE

        // ---- 压紧 (出仓前必做, 末态挡板保持降下) ----
        COMPACT_PRE_MOVE,    // 先把挡片转到前置位 (零点-30), 轮询到位 → 降挡板
        COMPACT_BAFFLE_DOWN, // 舵机2 降下, 等 settle
        COMPACT_PUSH,        // 同步带走到压紧角 (查表值), 轮询到位 → IDLE

        // ---- 出仓 (舵机2 不动, 已是降下); 新时序: 先转带到口, 停稳再顶料 ----
        DISP_BELT_TO_PORT,   // 同步带把头块送到出料口 (查表 count 位), 等够移动时间 → 舵机1 升起
        DISP_LIFT_UP,        // 舵机1 升起 settle 后, 等机械臂夹走 (notifyPicked 推进)
        DISP_LIFT_DOWN,      // 夹走后 舵机1 落下, 等 settle → 计数-1 → IDLE
    };

    // ========================================================================
    // 协作对象与运行时状态
    // ========================================================================
    ControlSerial* control_serial = nullptr;  // 共用的 Serial2 命令通道 (单例)

    State state = IDLE;                  // 当前状态
    uint32_t step_start_time = 0;        // 当前步骤起始时间 (settle / 超时基准)
    float belt_target_deg = 0.0f;        // 同步带目标角度 (到位判定)
    int material_count = 0;              // 仓内物料数量 (上层查表输入)

public:
    // 禁用拷贝和赋值
    TrayControl(const TrayControl&) = delete;
    TrayControl& operator=(const TrayControl&) = delete;

    /**
     * @brief 创建物料盘控制对象
     * @note  取 ControlSerial 单例 (与底盘/机械臂共用 Serial2);
     *        初始化两个舵机 LEDC PWM, 并置初始位 (顶料落下, 挡板升起)。
     */
    TrayControl() {
        this->control_serial = &(ControlSerial::get_instance());
        // 顶料舵机 PWM
        ledcSetup(kLiftChannel, kPwmFreq, kPwmResBits);
        ledcAttachPin(kLiftPin, kLiftChannel);
        // 挡板舵机 PWM
        ledcSetup(kBaffleChannel, kPwmFreq, kPwmResBits);
        ledcAttachPin(kBafflePin, kBaffleChannel);
        // 初始位: 顶料落下, 挡板升起
        liftDown();
        baffleUp();
    }

public:
    // ========================================================================
    // 第一层: 硬件原语 (不带时序, 一条命令对应一个硬件动作)
    // ========================================================================

    /**
     * @brief 同步带电机走到绝对角度 (位置控制)
     * @param deg 目标绝对角度 (度), 相对上电零点; 正=逆时针(让出出入口方向), 负=顺时针(推料/压紧方向)
     * @note  发一次即可, 电机内部闭环走到位; 到位判定靠 readBeltAngle 轮询。
     * @note  发完命令后留 kPositionReadDelayMs 间隔: 防与同帧随后的总线操作 (问询/别的命令)
     *        0 间隔背靠背把刚发的位置命令挤掉/错帧 (同夹取下降 bug 的总线时序问题)。
     */
    void beltRunToAngle(float deg) {
        this->control_serial->X_generate_set_position_command(kBeltAddr, deg, kBeltSpeed);
        this->control_serial->send_command();
        delay(kPositionReadDelayMs);   // 发后总线间隔, 防背靠背丢命令
    }

    /**
     * @brief 读同步带电机当前绝对位置
     * @param deg [out] 当前角度 (度); 失败时不变
     * @return true=读取成功, false=超时/校验失败
     * @note  先问询, 延迟 kPositionReadDelayMs 等回复再读; 全程持锁为原子事务。
     * @note  [ ! ] 8 号电机驱动器的 "响应" 须开启 (0x36 问询/响应协议), 否则读不到。
     */
    bool readBeltAngle(float& deg) {
        this->control_serial->thread_lock();
        this->control_serial->ask_current_position(kBeltAddr);
        delay(kPositionReadDelayMs);
        bool ok = this->control_serial->X_read_current_position(deg);
        this->control_serial->thread_unlock();
        return ok;
    }

    /// @brief 停止同步带电机
    void beltStop() {
        this->control_serial->thread_lock();
        this->control_serial->generate_stop_command(kBeltAddr);
        this->control_serial->send_command();
        this->control_serial->thread_unlock();
        delay(kPositionReadDelayMs);   // 发后总线间隔, 防背靠背丢命令
    }

    /// @brief 顶料舵机升起 - 进料用 (承接, 角度较高 40°)
    void liftUpStore() { setServoAngle(kLiftChannel, kLiftUpStoreAngle); }

    /// @brief 顶料舵机升起 - 出料用 (顶起物块, 角度较低 20°)
    void liftUpDispense() { setServoAngle(kLiftChannel, kLiftUpDispenseAngle); }

    /// @brief 顶料舵机落下 (初始位)
    void liftDown() { setServoAngle(kLiftChannel, kLiftDownAngle); }

    /// @brief 挡板舵机升起 (初始位, 让位)
    void baffleUp() { setServoAngle(kBaffleChannel, kBaffleUpAngle); }

    /// @brief 挡板舵机降下 (挡住物料定位)
    void baffleDown() { setServoAngle(kBaffleChannel, kBaffleDownAngle); }

public:
    // ========================================================================
    // 第二层: 组合动作 (启动即返回, 由 update() 推进; 不阻塞)
    // ========================================================================

    /**
     * @brief 入仓-放料前: 同步带挡片先退到入口位 (正角, 让出出入口), 到位后舵机1 再升起承接
     * @note  仅空闲时响应。新时序: 步进先走, 轮询读位置到接料位 (kEntryAngleDeg) 后才 liftUp,
     *        避免步进与舵机同时启动 (错开电流峰值 + 时序更明确)。之后机械臂放料, 再调 store()。
     */
    void prepareStore() {
        if (state != IDLE) {
            Serial.printf("[Tray] prepareStore IGNORED (state=%d, not IDLE)\n", state);
            return;
        }
        belt_target_deg = kEntryAngleDeg;   // 挡片退到正角, 让出入口空间 (只发步进, 不升舵机)
        Serial.printf("[Tray] prepareStore: belt -> entry %.1f (lift waits arrival)\n", belt_target_deg);
        beltRunToAngle(belt_target_deg);
        state = STORE_BELT_TO_ENTRY;
        step_start_time = millis();
    }

    /**
     * @brief 入仓-放料后: 舵机1 落下, 随后同步带挡片推回零点把料送进仓
     * @note  仅在等待放料态 (STORE_WAIT_PLACE) 响应; 推入到位后 material_count 自动 +1。
     */
    void store() {
        if (state != STORE_WAIT_PLACE) {
            return;
        }
        liftDown();
        state = STORE_LIFT_DOWN;
        step_start_time = millis();
    }

    /**
     * @brief 压紧 (出仓前必做): 先把挡片转到前置位 → 降挡板 → 同步带把当前物料推紧
     * @note  仅空闲时响应。前置: 挡片先转到 kCompactPreDeg (零点-30), 再降挡板,
     *        避免挡板下落时与物料干涉。压紧角按当前数量查内部表 kTightenTable[count]。
     *        末态挡板保持降下 (供随后出仓继续当挡板); 压紧到位时队首物料已到出料口。
     */
    void compact() {
        if (state != IDLE) {
            return;
        }
        belt_target_deg = kCompactPreDeg;   // 先转到前置位
        beltRunToAngle(belt_target_deg);
        state = COMPACT_PRE_MOVE;
        step_start_time = millis();
    }

    /**
     * @brief 出仓: 同步带先把头块送到出料口 → 停稳 → 舵机1 升起 → (机械臂夹走) → 落下
     * @note  仅空闲且仓非空时响应。前提是已先 compact (挡板已降下), 出仓全程不动挡板。
     *        新时序: 先转带子到 kTightenTable[count] (当前数量头块在口位), 带子停稳后再 liftUp,
     *        避免"一边顶料一边转带"。升起后停在 DISP_LIFT_UP 等 notifyPicked();
     *        机械臂夹走、舵机1 落下 settle 后 material_count 自动 -1。
     */
    void dispense() {
        if (state != IDLE || material_count <= 0) {
            Serial.printf("[Tray] dispense IGNORED (state=%d count=%d; need IDLE & count>0)\n",
                          state, material_count);
            return;
        }
        // 先转带子: 把当前 count 个物料的头块送到出料口 (不在此 liftUp)
        belt_target_deg = tightenAngleFor(material_count);
        Serial.printf("[Tray] dispense: belt -> port %.1f (count=%d), then lift\n",
                      belt_target_deg, material_count);
        beltRunToAngle(belt_target_deg);
        state = DISP_BELT_TO_PORT;
        step_start_time = millis();
    }

    /**
     * @brief 机械臂夹走完成通知 (推进出仓后半段)
     * @note  仅在 DISP_LIFT_UP (顶起等夹走) 时有效: 舵机1 落下, settle 后 count-1。
     */
    void notifyPicked() {
        if (state != DISP_LIFT_UP) {
            Serial.printf("[Tray] notifyPicked IGNORED (state=%d, not DISP_LIFT_UP)\n", state);
            return;
        }
        Serial.println("[Tray] notifyPicked: lift down");
        liftDown();
        state = DISP_LIFT_DOWN;
        step_start_time = millis();
    }

public:
    // ========================================================================
    // 查询 / 急停
    // ========================================================================

    /// @brief 是否有动作正在执行 (供上层判断忙闲)
    bool isBusy() const { return state != IDLE; }

    /// @brief 仓内当前物料数量 (上层查表的输入)
    int count() const { return material_count; }

    /**
     * @brief 手动物料数 +1 (由手柄 Y 键触发, 全局任意时刻可调)
     * @note  入仓不再自动计数, 改由人眼确认按 Y 加, 避免夹空导致计数虚高。
     *        出料仍自动 -1。不依赖状态机状态, 任何时候都生效。
     */
    void addMaterial() {
        material_count++;
        Serial.printf("[Tray] addMaterial (Y) -> count=%d\n", material_count);
    }

    /**
     * @brief 挡板(2号舵机)复位到升起位 (出仓结束后恢复初始态, 让出入口空间)
     * @note  compact() 降下挡板后保持降下; 放置全部完成(count=0)后须调此方法
     *        让挡板升回来, 否则下一轮入仓时挡板挡住进料。
     */
    void resetBaffle() {
        baffleUp();
        Serial.println("[Tray] resetBaffle: baffle up (ready for next store)");
    }

    /**
     * @brief 急停: 停同步带电机并复位状态机为 IDLE
     * @note  舵机本质是保持当前角度 (无反馈), 此处不强制复位舵机角度;
     *        如需归位由上层在停止后另行调用 liftDown()/baffleUp()。
     */
    void stop() {
        beltStop();
        state = IDLE;
    }

public:
    // ========================================================================
    // 每帧推进 (TaskCoordinator 每帧必调)
    // ========================================================================

    /**
     * @brief 状态机推进一帧
     * @param now 当前 millis() 时间戳
     * @note  等舵机 settle 用 now - step_start_time; 等同步带到位用 readBeltAngle 轮询;
     *        同步带移动带超时保护, 超时强制停车回 IDLE。
     */
    void update(uint32_t now) {
        switch (state) {
            case IDLE:
                break;

            // ---- 入仓: 同步带走到接料位, 读位置到位 → 升舵机1 (步进先走, 到位后舵机才升) ----
            case STORE_BELT_TO_ENTRY:
                if (beltArrived()) {
                    liftUpStore();   // 步进确认到位, 进料用较高角度升起承接
                    state = STORE_RETRACT;
                    step_start_time = now;
                    Serial.println("[Tray] belt at entry -> lift up (servo1)");
                }
                break;

            // ---- 入仓: 舵机1 升起 settle 后 → 等机械臂放料 ----
            case STORE_RETRACT:
                if (settled(now)) {
                    state = STORE_WAIT_PLACE;
                    step_start_time = now;
                    Serial.println("[Tray] lift up done -> WAIT_PLACE");
                }
                break;

            // ---- 入仓: 等机械臂放料 + 等 store() 调用; 此处不自动推进 ----
            case STORE_WAIT_PLACE:
                break;

            // ---- 入仓: 舵机1 落下 settle 后, 挡片推回零点把料送进仓 ----
            case STORE_LIFT_DOWN:
                if (settled(now)) {
                    belt_target_deg = kStoreInDeg;   // 推回零点 (= 物料推入位)
                    beltRunToAngle(belt_target_deg);
                    state = STORE_PUSH;
                    step_start_time = now;
                    Serial.printf("[Tray] STORE_LIFT_DOWN done -> STORE_PUSH belt->%.1f\n", belt_target_deg);
                }
                break;

            // ---- 入仓: 同步带推进等够时间 → 结束 (计数改由 Y 键手动加, 此处不自动+1) ----
            case STORE_PUSH:
                if (beltMoved(now)) {
                    state = IDLE;
                    Serial.printf("[Tray] STORE_PUSH done -> IDLE (count manual via Y, =%d)\n", material_count);
                }
                break;

            // ---- 压紧: 挡片转到前置位, 等够移动时间 → 降挡板 ----
            case COMPACT_PRE_MOVE:
                if (beltMoved(now)) {
                    baffleDown();
                    state = COMPACT_BAFFLE_DOWN;
                    step_start_time = now;
                    Serial.println("[Tray] COMPACT_PRE_MOVE done -> baffle down");
                }
                break;

            // ---- 压紧: 挡板降下 settle 后, 同步带走到压紧角 (查表) ----
            case COMPACT_BAFFLE_DOWN:
                if (settled(now)) {
                    belt_target_deg = tightenAngleFor(material_count);
                    beltRunToAngle(belt_target_deg);
                    state = COMPACT_PUSH;
                    step_start_time = now;
                    Serial.printf("[Tray] COMPACT push belt->%.1f (count=%d)\n", belt_target_deg, material_count);
                }
                break;

            // ---- 压紧: 同步带等够移动时间 → 结束 (挡板保持降下) ----
            case COMPACT_PUSH:
                if (beltMoved(now)) {
                    state = IDLE;
                    Serial.println("[Tray] COMPACT done -> IDLE (baffle stays down)");
                }
                break;

            // ---- 出仓: 同步带把头块送到出料口, 等够移动时间 → 停稳后舵机1 升起 ----
            case DISP_BELT_TO_PORT:
                if (beltMoved(now)) {
                    liftUpDispense();   // 带子已停稳, 出料用较低角度顶起物块
                    state = DISP_LIFT_UP;
                    step_start_time = now;
                    Serial.println("[Tray] DISP belt at port -> lift up");
                }
                break;

            // ---- 出仓: 舵机1 升起 settle 后, 等机械臂夹走 (notifyPicked 推进) ----
            case DISP_LIFT_UP:
                // settle 后停此等 notifyPicked; settle 期间也不自动推进 (等人/机械臂)
                break;

            // ---- 出仓: 舵机1 落下 settle 后 → 计数-1 → 结束 ----
            case DISP_LIFT_DOWN:
                if (settled(now)) {
                    if (material_count > 0) {
                        material_count--;
                    }
                    state = IDLE;
                    Serial.printf("[Tray] DISP done -> IDLE, count=%d\n", material_count);
                }
                break;
        }
    }

private:
    // ========================================================================
    // 状态机辅助
    // ========================================================================

    /// @brief 舵机动作是否已等够 settle 时间
    bool settled(uint32_t now) const {
        return (now - step_start_time) >= kServoSettleMs;
    }

    /// @brief 当前步骤是否超过给定超时
    bool timedOut(uint32_t now, uint32_t limit) const {
        return (now - step_start_time) > limit;
    }

    /**
     * @brief 按物料数量查压紧角度表
     * @param n 物料数量 (索引)
     * @return 该数量对应的步进电机绝对角度 (度); 越界则钳到表两端
     * @note  表见 kTightenTable, 占位待测。
     */
    float tightenAngleFor(int n) const {
        const int maxIndex = static_cast<int>(sizeof(kTightenTable) / sizeof(kTightenTable[0])) - 1;
        if (n < 0) n = 0;
        if (n > maxIndex) n = maxIndex;
        return kTightenTable[n];
    }

    /// @brief 同步带是否已等够移动时间 (闭环步进发命令后, 等 kBeltMoveMs 即视为到位, 不读位置)
    bool beltMoved(uint32_t now) const {
        return (now - step_start_time) >= kBeltMoveMs;
    }

    /// @brief 同步带是否走到目标角度 (读位置且在阈值内; 当前流程已改用 beltMoved 固定时间, 此函数备用)
    bool beltArrived() {
        float pos = 0.0f;
        return readBeltAngle(pos) &&
               fabsf(pos - belt_target_deg) <= kPosArrivedThreshDeg;
    }

private:
    /**
     * @brief 驱动舵机到指定角度 (LEDC PWM)
     * @param channel LEDC 通道 (kLiftChannel / kBaffleChannel)
     * @param angle   目标角度 (度), 限幅到 [0, 180]
     * @note  角度 → 脉宽(us) → LEDC 占空比, 同 ArmControl::setGripperAngle。
     */
    void setServoAngle(int channel, float angle) {
        if (angle < 0.0f) angle = 0.0f;
        if (angle > 180.0f) angle = 180.0f;
        float pulse_us = kMinPulseUs + (kMaxPulseUs - kMinPulseUs) * (angle / 180.0f);
        const float period_us = 1000000.0f / kPwmFreq;
        const uint32_t max_duty = (1u << kPwmResBits) - 1u;
        uint32_t duty = static_cast<uint32_t>((pulse_us / period_us) * max_duty);
        ledcWrite(channel, duty);
    }
};

// static constexpr 数组成员的类外定义 (C++14 及以前要求): 运行时被 tightenAngleFor 按下标
// 取值 (ODR-use), 需要这条定义提供存储, 否则链接报 "undefined reference"。本项目头文件仅被
// main.cpp 单一翻译单元包含, 不存在重复定义问题。(C++17 起 static constexpr 隐含 inline 可省略。)
constexpr float TrayControl::kTightenTable[10];
