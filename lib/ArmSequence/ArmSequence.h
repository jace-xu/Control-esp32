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
 * @note  == Version 2.0.0 ==
 */
class ArmSequence {
private:
    // ========================================================================
    // 上下电机状态机 (两阶段统一位置控制)
    // 阶段1 (A键夹取, MODE_RECORD):
    //   [PRE_CATCH(L1预备摆位)] → PRE_ALIGN(先降一小段+PID对准,等A确认)
    //   → DESCENDING(位置下降到夹取位) → GRIPPING(舵机夹取+settle) → ASCENDING(位置回升) → IDLE
    // 阶段2 (X键放置, MODE_CONSUME):
    //   PRE_ALIGN(先降一小段+PID对准,等X确认) → DESCENDING(位置下降到放置位)
    //   → GRIPPING(release放料+等待) → ASCENDING(位置回升) → IDLE (队列非空则跑下一色)
    // ========================================================================
    enum VerticalState {
        IDLE,         // 空闲: 机械臂未在执行上下动作
        PRE_CATCH,    // [阶段1 前置] 预备摆位: 按 L1 后角度电机转到 kPreCatchAngleDeg, 等按 A 进 PRE_ALIGN
        PRE_ALIGN,    // 对准等待: 上下已先降一小段, 角度+前后跑PID对准, 等确认键 (阶段1=A, 阶段2=X)
        DESCENDING,   // 下降中: 两阶段均位置控制 (阶段1→夹取位, 阶段2→放置位)
        GRIPPING,     // 底部动作: 阶段1=舵机夹取+settle; 阶段2=release放料+等待
        ASCENDING,    // 回升中: 两阶段均位置控制
    };

    // ========================================================================
    // 任务模式: 决定本次/本轮序列的行为
    // ========================================================================
    enum SequenceMode {
        MODE_NONE,    // 无任务
        MODE_RECORD,  // 阶段1 (A键): 请求误差1, 握手后记录一个颜色入队, 跑完整序列
        MODE_CONSUME, // 阶段2 (X键): 请求误差2, 按队列 FIFO 逐个颜色跑序列直到队空
    };


    // ========================================================================
    // 上下电机自动序列参数 (按住 A 键时执行)
    // 序列: 下降 → 夹取等待 → 回升 (位置为相对上电零点的绝对角)
    // [ ! ] 圈数带符号: 正=下降约定。setPosition 原样下发, 方向反了改这里符号即可。
    //       目标绝对角度 = 圈数常量 × 360。
    // ========================================================================
    static constexpr float kVerticalDirection = 1.0f;  // 上下方向因子: 正=下降, 负=上升
    static constexpr float kRotationDirection = 1.0f;  // 旋转方向因子: 正=逆时针, 负=顺时针

    static constexpr float kDescendTargetRotations = 5.0f*kVerticalDirection;  // 阶段1 下降到夹取位: 5 圈
    static constexpr float kAscendTargetRotations = 1.0f*kVerticalDirection;   // 阶段1 回升目标: 1 圈
    static constexpr uint32_t kGripDelayMs = 5000;           // 底部放置等待时间: 5 秒

    // 阶段2 放置位置目标 (与阶段1 夹取位区分; 圈数带符号同 kVerticalDirection 约定)
    static constexpr float kPlaceDescendRotations = 5.0f*kVerticalDirection;   // 阶段2 下降到放置位: 5 圈
    static constexpr float kPlaceAscendRotations = 1.0f*kVerticalDirection;    // 阶段2 放置后回升: 1 圈

    // 超时保护 (位置反馈失败/电机卡死时强制停止; 阶段1、阶段2 共用)
    static constexpr uint32_t kDescentTimeoutMs = 15000; // 下降超时保护: 15 秒
    static constexpr uint32_t kAscentTimeoutMs = 12000;  // 回升超时保护: 12 秒

    // 视觉串流握手重传参数: START 命令可能被干扰丢失, 在收到首帧数据前定时重发
    static constexpr uint32_t kStartRetryIntervalMs = 300;  // 未收到数据则每 300ms 重发 START
    static constexpr uint8_t kStartMaxRetries = 10;         // START 最大重传次数 (约 3 秒)

    // 阶段1 位置控制新参数
    static constexpr float kPreDescendRotations = 0.5f*kVerticalDirection;    // 第一次按A先降0.5圈 
    static constexpr uint32_t kGripSettleMs = 800;          // 舵机夹取到位等待时间
    static constexpr float kPositionArrivedThreshDeg = 5.0f;// 位置到位判定阈值 (度)
    static constexpr uint32_t kPreAlignTimeoutMs = 30000;   // PRE_ALIGN 等待第二次按A的超时 (30秒)

    // PRE_CATCH 预备摆位参数 (阶段1 前置, 按 L1 触发)
    static constexpr float kPreCatchAngleDeg = 10.0f*kRotationDirection;       // 角度电机预备摆位目标绝对角
    static constexpr uint32_t kPreCatchTimeoutMs = 30000;   // PRE_CATCH 等待按A的超时 (30秒)

    // PID 收敛判定 (对准完成后才能按 A 确认)
    static constexpr uint8_t kConvergeFramesNeeded = 10;    // 连续入阈帧数 (50Hz ≈ 0.2s)
    static constexpr float kConvergeAngleThresh = 8.0f;     // 角度轴误差收敛阈值 (像素, 树莓派视觉输出单位)
    static constexpr float kConvergeForwardThresh = 8.0f;   // 前后轴误差收敛阈值 (像素, 树莓派视觉输出单位)

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
    float vertical_target_degrees = 0.0f;   // 上下电机最近一次 setVerticalPosition 的目标角度 (用于到位判定)
    uint8_t converge_count = 0;             // PID 收敛连续入阈帧计数
    bool servo_stopped = false;             // 伺服电机是否已停 (避免非串流期间每帧重发停止命令抢总线)

    // 视觉伺服内部标志 (原为函数内 static 变量)
    bool timeout_warned = false;            // "无有效视觉数据" 警告是否已打印过
    uint32_t last_debug_print = 0;          // 上次打印视觉误差调试信息的时间戳

    // 视觉串流握手标志: 树莓派仅在 ESP32 请求后才发误差数据
    bool vision_streaming = false;          // 当前是否已请求树莓派发送误差数据
    bool stream_confirmed = false;          // 是否已收到首帧有效数据 (确认对接成功)
    uint32_t last_start_sent_time = 0;      // 上次发送 START 命令的时间戳 (用于超时重传)
    uint8_t start_retry_count = 0;          // START 已重传次数
    bool start_failed_warned = false;       // 重传耗尽的失败警告是否已打印过

    // 本次串流请求的误差类型与颜色 (重传时需重发同一条命令)
    uint8_t stream_error_type = 1;          // 1=误差1, 2=误差2(按颜色)
    uint8_t stream_color = 0;               // 误差2 的目标颜色(1=绿/2=蓝/3=红); 误差1 时为 0

    // 任务模式与颜色队列
    SequenceMode mode = MODE_NONE;          // 当前任务模式
    bool color_recorded_this_run = false;   // 本轮(阶段1)是否已记录过颜色, 防止重复入队

    // 颜色队列容量 (阶段1 最多可记录多少个待抓物料)
    static constexpr uint8_t kColorQueueCapacity = 16;

    uint8_t color_queue[kColorQueueCapacity]; // 颜色环形队列
    uint8_t queue_head = 0;                 // 队首索引 (出队位置)
    uint8_t queue_tail = 0;                 // 队尾索引 (入队位置)
    uint8_t queue_count = 0;                // 队列当前元素个数

    // 触发边沿检测: A 键(记录) 与 X 键(消费) 各自检测上升沿
    bool prev_a_pressed = false;            // 上一帧 A 键是否按住
    bool prev_x_pressed = false;            // 上一帧 X 键是否按住
    bool prev_l1_pressed = false;           // 上一帧 L1 键是否按住 (PRE_CATCH 预备摆位触发)

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

    /// @brief 当前颜色队列中待抓物料数量
    uint8_t queuedColorCount() const { return queue_count; }

    /**
     * @brief 每帧调用的主入口 (两阶段 + 颜色队列模式)
     * @param aPressed     A 键: 阶段1, 记录一个颜色入队并跑完整序列
     * @param xPressed     X 键: 阶段2, 按队列 FIFO 逐个颜色跑序列直到队空
     * @param abortPressed B 键: 中止当前运行, 复位为 IDLE
     * @param l1Pressed    L1 键: 阶段1 前置, 空闲时角度电机预备摆位 (PRE_CATCH)
     * @details A / X / L1 上升沿在空闲时各自触发; 序列自锁运行 (松开不影响)。
     *          L1 为可选预备步骤: IDLE 按 L1 → PRE_CATCH 摆角度 → 按 A 进 PRE_ALIGN;
     *          也可在 IDLE 直接按 A 跳过 PRE_CATCH。运行期间按 B 立即停止。
     */
    void update(bool aPressed, bool xPressed, bool abortPressed, bool l1Pressed) {
        const uint32_t currentTime = millis();
        bool aRising = aPressed && !prev_a_pressed;
        bool xRising = xPressed && !prev_x_pressed;
        bool l1Rising = l1Pressed && !prev_l1_pressed;

        // 中止: 序列运行期间按下 B 键 → 立即停止所有动作并复位为 IDLE
        if (active && abortPressed) {
            Serial.println("Sequence aborted by user");
            // 阶段1 回滚: 本轮若已记录颜色则丢弃 (当作这一轮没发生),
            // 并松开夹爪 (无论是在 PRE_ALIGN 等待, 还是已下降/夹取/回升)。
            // 必须在 stop() 之前做: stop()→haltAndReset() 会把 mode 复位为 MODE_NONE。
            if (mode == MODE_RECORD) {
                if (color_recorded_this_run) {
                    dropLastRecordedColor();
                }
                if (arm != nullptr) {
                    arm->release();   // 松开夹爪, 丢回可能已夹住的物料
                }
            }
            stop();
            prev_a_pressed = aPressed;
            prev_x_pressed = xPressed;
            prev_l1_pressed = l1Pressed;
            return;
        }

        // 启动 (仅空闲时响应上升沿): A → 阶段1直接对准, X → 阶段2, L1 → 阶段1预备摆位
        if (!active) {
            if (aRising) {
                startRecordSequence(currentTime);
            } else if (xRising) {
                startConsumeStage(currentTime);
            } else if (l1Rising) {
                startPreCatch(currentTime);
            }
        }
        // 阶段1: PRE_CATCH 期间按 A → 请求误差1并进入 PRE_ALIGN (不强制等角度电机到位)
        else if (state == PRE_CATCH && aRising) {
            Serial.println("[Stage 1] A-key: leaving pre-catch, entering pre-align");
            startVisionStream(currentTime, 1, kColorNone);  // 请求误差1
            arm->resetPID();
            enterPreAlign(currentTime);
        }
        // PRE_ALIGN 期间按确认键 → 仅当已收敛时才确认并下降; 未收敛则提示。
        // 确认键按 mode 区分: 阶段1=A, 阶段2=X (与各自入口键一致)。
        else if (state == PRE_ALIGN &&
                 ((mode == MODE_RECORD && aRising) || (mode == MODE_CONSUME && xRising))) {
            if (converge_count >= kConvergeFramesNeeded) {
                confirmAlignAndDescend(currentTime);
            } else {
                Serial.printf("Not aligned yet (conv=%u/%u), confirm key ignored\n",
                              converge_count, kConvergeFramesNeeded);
            }
        }

        prev_a_pressed = aPressed;
        prev_x_pressed = xPressed;
        prev_l1_pressed = l1Pressed;

        // 自锁运行: 序列一旦启动, 不再依赖按键是否按住
        if (active) {
            if (vision_streaming) {
                retryStartIfNeeded(currentTime);  // 未收到数据则超时重传 START
                runVisualServo(currentTime);
                servo_stopped = false;            // 伺服正在运行, 下次进入非串流时需再停一次
            } else if ((state == DESCENDING || state == ASCENDING) && !servo_stopped) {
                // 下降/回升期 (非串流, 两阶段统一位置控制) 停一次伺服即可。
                // [ ! ] 必须排除 PRE_CATCH/PRE_ALIGN: stopServo() 会给角度电机(地址5)发停止命令,
                //       会打断 PRE_CATCH 的角度电机位置移动; PRE_ALIGN 靠 vision_streaming 跑伺服不入此分支。
                // 每帧重发停止命令还会与地址6 的位置问询/响应在同一 Serial2 上交错, 拖慢位置读取。
                arm->stopServo();
                servo_stopped = true;
            }
            runVerticalSequence(currentTime);
        }
    }

    /**
     * @brief 立即停止所有机械臂电机并复位状态机
     * @note  中止 / 紧急停止 / 手柄断连超时时调用。
     * @note  仅复位运行状态与模式; 颜色队列保留 (中止阶段1不清空已记录的颜色)。
     */
    void stop() {
        haltAndReset();
    }

    /// @brief 清空颜色队列 (供 main 在需要时手动复位任务)
    void clearColorQueue() {
        queue_head = 0;
        queue_tail = 0;
        queue_count = 0;
    }

private:
    // ========================================================================
    // 统一的停止原语
    // ========================================================================

    /**
     * @brief 停所有机械臂电机并复位 PID, 通知树莓派停止串流
     * @note  仅停电机, 不动状态机/自锁标志。供"正常跑完后回静止"与"彻底结束"复用。
     */
    void haltMotors() {
        stopVisionStream();      // 通知树莓派停止发送误差数据
        if (arm != nullptr) {
            arm->stop();         // 停三个电机 (角度/上下/前后) 并复位 PID
        }
    }

    /**
     * @brief 复位状态机与自锁标志 (回到空闲, 不碰电机/队列)
     */
    void resetRunState() {
        state = IDLE;
        active = false;
        mode = MODE_NONE;
    }

    /**
     * @brief 彻底停止: 停电机 + 复位状态机 + 解除自锁
     * @note  中止(B键) / 超时保护 / 队列耗尽 共用同一条停止路径, 避免多份实现不一致。
     * @note  颜色队列不在此清空 (中止阶段1保留已记录的颜色)。
     */
    void haltAndReset() {
        haltMotors();
        resetRunState();
    }

    // ========================================================================
    // 颜色队列操作 (FIFO 环形缓冲)
    // ========================================================================

    /// @brief 颜色入队 (队满则丢弃并打印警告)
    void enqueueColor(uint8_t color) {
        if (queue_count >= kColorQueueCapacity) {
            Serial.println("WARNING: Color queue full, dropping color");
            return;
        }
        color_queue[queue_tail] = color;
        queue_tail = (queue_tail + 1) % kColorQueueCapacity;
        queue_count++;
        Serial.printf("Recorded color %u, queue size = %u\n", color, queue_count);
    }

    /// @brief 颜色出队 (取队首; 队空返回 false)
    bool dequeueColor(uint8_t& color) {
        if (queue_count == 0) {
            return false;
        }
        color = color_queue[queue_head];
        queue_head = (queue_head + 1) % kColorQueueCapacity;
        queue_count--;
        return true;
    }

    /// @brief 撤销本轮最后一次入队的颜色 (阶段1 中止回滚用)
    /// @note  FIFO 环形缓冲, 回退队尾指针即可丢弃最新入队的元素。
    ///        阶段1 每轮至多入队一个且不出队, 故队尾元素必为本轮所记颜色。
    void dropLastRecordedColor() {
        if (queue_count == 0) {
            return;
        }
        queue_tail = (queue_tail + kColorQueueCapacity - 1) % kColorQueueCapacity;
        queue_count--;
        Serial.printf("Aborted: dropped last recorded color, queue size = %u\n", queue_count);
    }

    // ========================================================================
    // 序列启动
    // ========================================================================

    /**
     * @brief 阶段1启动 (A 键上升沿): 请求误差1, 握手后记录一个颜色, 跑完整升降序列
     * @param currentTime 当前 millis() 时间戳
     */
    void startRecordSequence(uint32_t currentTime) {
        mode = MODE_RECORD;
        color_recorded_this_run = false;          // 本轮尚未记录颜色
        Serial.println("[Stage 1] Record: requesting error1");
        beginVerticalSequence(currentTime, 1, kColorNone);  // 误差1, 无颜色
    }

    /**
     * @brief PRE_CATCH 预备摆位启动 (L1 上升沿, 仅空闲): 角度电机转到预备角, 等按 A 进 PRE_ALIGN
     * @param currentTime 当前 millis() 时间戳
     * @note  阶段1 前置步骤: 自锁但不开视觉、不跑 PID; 只发一次角度位置命令。
     */
    void startPreCatch(uint32_t currentTime) {
        active = true;
        mode = MODE_RECORD;                 // PRE_CATCH 属于阶段1
        color_recorded_this_run = false;    // 本轮尚未记录颜色
        Serial.printf("[Stage 1] Pre-catch: angle motor to %.1f deg, waiting for A\n", kPreCatchAngleDeg);
        arm->setAnglePosition(kPreCatchAngleDeg);  // 角度电机预备摆位 (带符号绝对角)
        descent_start_time = currentTime;   // 复用作 PRE_CATCH 超时基准
        state = PRE_CATCH;
    }

    /**
     * @brief 阶段2启动 (X 键上升沿): 从队列取首个颜色开始消费, 队空则不启动
     * @param currentTime 当前 millis() 时间戳
     */
    void startConsumeStage(uint32_t currentTime) {
        uint8_t color = 0;
        if (!dequeueColor(color)) {
            Serial.println("[Stage 2] Consume: color queue empty, nothing to do");
            return;
        }
        mode = MODE_CONSUME;
        Serial.printf("[Stage 2] Consume: requesting error2 for color %u (%u left)\n",
                      color, queue_count);
        beginVerticalSequence(currentTime, 2, color);  // 误差2, 指定颜色
    }

    /**
     * @brief 启动一次升降序列 (两阶段统一: 先降一小段 + PID 对准, 等确认键)
     *        阶段1 (MODE_RECORD): 误差1, 进入 PRE_ALIGN 等 A 确认
     *        阶段2 (MODE_CONSUME): 误差2(按颜色), 进入 PRE_ALIGN 等 X 确认
     * @param currentTime 当前 millis() 时间戳
     * @param errorType   1=误差1, 2=误差2
     * @param color       误差2 的目标颜色; 误差1 传 kColorNone
     */
    void beginVerticalSequence(uint32_t currentTime, uint8_t errorType, uint8_t color) {
        active = true;
        startVisionStream(currentTime, errorType, color);  // 请求串流 (带超时重传)
        arm->resetPID();
        // 两阶段统一: 都先降一小段 + 跑 PID 对准, 等确认键 (阶段1=A, 阶段2=X)
        enterPreAlign(currentTime);
    }

    /**
     * @brief 进入 PRE_ALIGN: 上下位置先降一小段, 同时跑角度+前后 PID 对准, 等第二次按 A
     * @param currentTime 当前 millis() 时间戳
     * @note  供两处复用: IDLE+A 直接进阶段1 (beginVerticalSequence), 以及 PRE_CATCH+A 转入。
     * @note  调用前视觉串流应已请求 (beginVerticalSequence 已调 startVisionStream;
     *        PRE_CATCH 转入时需自行先请求误差1)。
     */
    void enterPreAlign(uint32_t currentTime) {
        Serial.printf("Pre-descend %.2f rotations, waiting for A-confirm\n", kPreDescendRotations);
        float preTargetDeg = kPreDescendRotations * 360.0f;  // 符号已在常量里 (正=下降)
        vertical_target_degrees = preTargetDeg;
        arm->setVerticalPosition(preTargetDeg);
        converge_count = 0;  // 从零开始重新判定收敛
        descent_start_time = currentTime;  // 复用作 PRE_ALIGN 超时基准
        state = PRE_ALIGN;
    }

    /**
     * @brief PRE_ALIGN 确认 → 停视觉伺服 + 位置下降到目标 (按 mode 选夹取位/放置位)
     * @param currentTime 当前 millis() 时间戳
     * @note  阶段1 下降到夹取位 kDescendTargetRotations; 阶段2 下降到放置位 kPlaceDescendRotations。
     *        下降期 vision_streaming=false, 仅地址6 位置轮询, 无伺服抢总线。
     */
    void confirmAlignAndDescend(uint32_t currentTime) {
        const char* tag = (mode == MODE_RECORD) ? "[Stage 1]" : "[Stage 2]";
        Serial.printf("%s confirm: aligned, descending to target position\n", tag);
        stopVisionStream();       // 下降期间停视觉伺服
        arm->stopServo();
        float rotations = (mode == MODE_RECORD) ? kDescendTargetRotations : kPlaceDescendRotations;
        float targetDeg = rotations * 360.0f;  // 符号已在常量里 (正=下降)
        vertical_target_degrees = targetDeg;
        arm->setVerticalPosition(targetDeg);
        state = DESCENDING;
        descent_start_time = currentTime;
    }

    /**
     * @brief 本轮升降序列正常跑完后的推进逻辑
     * @param currentTime 当前 millis() 时间戳
     * @details 阶段2(消费): 队列还有颜色 → 自动启动下一个颜色的序列; 队空 → 结束。
     *          阶段1(记录): 一轮记一个颜色, 跑完即结束, 等待下次按 A。
     */
    void finishRun(uint32_t currentTime) {
        // 先停掉本轮的串流与电机 (回到静止)
        haltMotors();

        if (mode == MODE_CONSUME) {
            uint8_t color = 0;
            if (dequeueColor(color)) {
                // 队列还有物料: 直接启动下一个颜色的序列 (自锁继续)
                Serial.printf("[Stage 2] Next color %u (%u left)\n", color, queue_count);
                beginVerticalSequence(currentTime, 2, color);
                return;
            }
            Serial.println("[Stage 2] Consume: queue drained, all done");
        }

        // 阶段1 握手失败 → 整轮没记到颜色, 提示用户 (队列未增长)
        if (mode == MODE_RECORD && !color_recorded_this_run) {
            Serial.println("[Stage 1] WARNING: no color recorded this run (no vision data)");
        }

        // 阶段1 单轮完成, 或阶段2 队列耗尽: 彻底结束, 解除自锁
        resetRunState();
    }

    /// @brief 请求树莓派开始发送视觉误差数据 (幂等: 已在串流则不重发)
    /// @param currentTime 当前 millis() 时间戳, 用于初始化重传计时
    /// @param errorType   误差类型: 1=误差1, 2=误差2(按颜色)
    /// @param color       误差2 的目标颜色(1=绿/2=蓝/3=红); 误差1 传 0
    void startVisionStream(uint32_t currentTime, uint8_t errorType, uint8_t color) {
        if (!vision_streaming) {
            stream_error_type = errorType;
            stream_color = color;
            vision_streaming = true;
            stream_confirmed = false;       // 等待首帧数据确认对接
            last_start_sent_time = currentTime;
            start_retry_count = 0;
            start_failed_warned = false;
            sendStartCommand();
        }
    }

    /// @brief 按当前 stream_error_type/stream_color 发送对应的 START 命令
    void sendStartCommand() {
        if (stream_error_type == 2) {
            vision->requestStart2(stream_color);
        } else {
            vision->requestStart1();
        }
    }

    /// @brief 通知树莓派停止发送视觉误差数据 (幂等: 未串流则不重发)
    void stopVisionStream() {
        if (vision_streaming) {
            vision->requestStop();
            vision_streaming = false;
            stream_confirmed = false;
        }
    }

    /**
     * @brief START 命令超时重传检查
     * @param currentTime 当前 millis() 时间戳
     * @details 已请求串流但尚未收到首帧数据时, 每隔 kStartRetryIntervalMs 重发一次
     *          START, 防止握手帧被干扰丢失导致树莓派始终不发数据。收到首帧后
     *          (stream_confirmed=true) 停止重传; 超过最大次数后放弃并打印一次警告。
     * @note  边降边对准模式: 重传耗尽不中止序列, 机械臂继续按时间序列动作 (可能盲抓),
     *        仅打印警告提示握手失败。
     */
    void retryStartIfNeeded(uint32_t currentTime) {
        if (!vision_streaming || stream_confirmed) {
            return;  // 未串流, 或已确认对接, 无需重传
        }
        if (start_retry_count >= kStartMaxRetries) {
            if (!start_failed_warned) {
                Serial.println("ERROR: Vision handshake failed after max retries, running blind!");
                start_failed_warned = true;
            }
            return;  // 已达重传上限, 放弃重传
        }
        if ((currentTime - last_start_sent_time) >= kStartRetryIntervalMs) {
            sendStartCommand();
            last_start_sent_time = currentTime;
            start_retry_count++;
            Serial.printf("Vision handshake: resending START (retry %u/%u)\n",
                          start_retry_count, kStartMaxRetries);
        }
    }

    /**
     * @brief 视觉伺服: 角度 + 前后 PID 闭环控制
     * @param currentTime 当前 millis() 时间戳
     */
    void runVisualServo(uint32_t currentTime) {
        // 从视觉串口读取树莓派发来的误差数据
        VisionError visionError = vision->read();

        // 仅接受与本次请求误差类型匹配的帧 (id1: 1=误差1, 2=误差2)。
        // 切换串流 (STOP→START) 的瞬间, 接收缓冲里可能残留上一类型的旧帧,
        // 若不校验会导致: 提前误判握手成功、记错颜色、用旧误差驱动 PID。
        bool frameMatches = visionError.valid && (visionError.id1 == stream_error_type);

        if (frameMatches) {
            // 类型匹配: 确认握手成功 (停止 START 重传), 清除超时警告, 执行 PID 伺服
            stream_confirmed = true;
            timeout_warned = false;

            // 阶段1: 握手成功后记录本轮第一个颜色入队 (每轮只记一个)
            if (mode == MODE_RECORD && !color_recorded_this_run) {
                enqueueColor(visionError.id2);
                color_recorded_this_run = true;
            }

            arm->updateVisualServo(visionError.angleError, visionError.forwardError);

            // PID 收敛判定 (PRE_ALIGN 期间, 两阶段都需对准):
            // 角度与前后误差同时入阈才计数, 任一超阈值则清零。
            // 连续 kConvergeFramesNeeded 帧入阈后, update() 才允许按确认键 (阶段1=A, 阶段2=X)。
            if (state == PRE_ALIGN) {
                if (fabsf(visionError.angleError) <= kConvergeAngleThresh &&
                    fabsf(visionError.forwardError) <= kConvergeForwardThresh) {
                    if (converge_count < kConvergeFramesNeeded) {
                        converge_count++;
                        if (converge_count == kConvergeFramesNeeded) {
                            // 确认键按 mode 区分: 阶段1=A(夹取), 阶段2=X(放置)
                            if (mode == MODE_RECORD) {
                                Serial.println("[Stage 1] Aligned! Press A to confirm grip.");
                            } else {
                                Serial.println("[Stage 2] Aligned! Press X to confirm place.");
                            }
                        }
                    }
                } else {
                    if (converge_count >= kConvergeFramesNeeded) {
                        const char* tag = (mode == MODE_RECORD) ? "[Stage 1]" : "[Stage 2]";
                        Serial.printf("%s Alignment lost, waiting to re-converge...\n", tag);
                    }
                    converge_count = 0;
                }
            }

            // 调试输出: 每 200ms 打印一次视觉误差值
            if ((currentTime - last_debug_print) > 200) {
                Serial.printf("Vision: angle=%.2f, forward=%.2f conv=%u/%u\n",
                              visionError.angleError,
                              visionError.forwardError,
                              converge_count, kConvergeFramesNeeded);
                last_debug_print = currentTime;
            }
        } else {
            // 无有效帧 (串口无数据 / 解析失败 / 数据超时 / 误差类型不匹配):
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
     *        DESCENDING → GRIPPING → ASCENDING → IDLE(结束)
     * @param currentTime 当前 millis() 时间戳
     * @note  启动逻辑在 beginVerticalSequence() (A/X 上升沿触发), 故运行期不会停在 IDLE。
     */
    void runVerticalSequence(uint32_t currentTime) {
        switch (state) {
            case IDLE:
                break;

            // ---- PRE_CATCH (阶段1 前置): 角度电机已发位置命令, 等按 A; 此处仅超时防护 ----
            case PRE_CATCH:
                // 按 A 转入 PRE_ALIGN 由 update() 处理 (不强制等角度电机到位)。
                // 超时防护: 用户按 L1 后迟迟不按 A, 避免无限期挂着。
                if ((currentTime - descent_start_time) > kPreCatchTimeoutMs) {
                    Serial.println("ERROR: Pre-catch timeout (no A-press)! Stopping.");
                    haltAndReset();
                }
                break;

            // ---- PRE_ALIGN (仅阶段1): 等第二次按A, update() 已处理按键, 此处作超时防护 ----
            case PRE_ALIGN:
                // 对准期间视觉伺服继续在 runVisualServo 中运行;
                // 第二次按 A 由 update() 处理并转变到 DESCENDING。
                // 超时防护: 用户迟迟不按第二次 A 时, 避免视觉串流与电机无限期挂着。
                if ((currentTime - descent_start_time) > kPreAlignTimeoutMs) {
                    Serial.println("ERROR: Pre-align timeout (no second A-press)! Stopping.");
                    haltAndReset();
                }
                break;

            // ---- DESCENDING: 两阶段统一位置控制, 位置反馈轮询到位 ----
            case DESCENDING: {
                float currentPos = 0.0f;
                if (arm->readVerticalPosition(currentPos)) {
                    if (fabsf(currentPos - vertical_target_degrees) <= kPositionArrivedThreshDeg) {
                        if (mode == MODE_RECORD) {
                            Serial.printf("Descent complete (pos=%.1f), gripping\n", currentPos);
                            arm->grip();    // 阶段1=夹取: 到位夹紧物料
                        } else {
                            Serial.printf("Descent complete (pos=%.1f), releasing\n", currentPos);
                            arm->release(); // 阶段2=放置: 到位松开夹爪放下物料
                        }
                        state = GRIPPING;
                        grip_start_time = currentTime;
                    }
                }
                // 超时保护 (位置反馈失败或电机卡死)
                if ((currentTime - descent_start_time) > kDescentTimeoutMs) {
                    Serial.println("ERROR: Descent timeout! Stopping.");
                    haltAndReset();
                }
                break;
            }

            // ---- GRIPPING: 阶段1=夹取settle后回升; 阶段2=放置等待后回升 ----
            case GRIPPING: {
                // 阶段1 等舵机夹紧 settle, 阶段2 等放置稳定; 时间到则位置回升
                uint32_t waitMs = (mode == MODE_RECORD) ? kGripSettleMs : kGripDelayMs;
                if ((currentTime - grip_start_time) >= waitMs) {
                    float rotations = (mode == MODE_RECORD) ? kAscendTargetRotations
                                                            : kPlaceAscendRotations;
                    float ascendTargetDeg = rotations * 360.0f;  // 符号已在常量里 (正=下降)
                    Serial.printf("Wait done (%lums), ascending to %.1f deg\n", waitMs, ascendTargetDeg);
                    vertical_target_degrees = ascendTargetDeg;
                    arm->setVerticalPosition(ascendTargetDeg);
                    state = ASCENDING;
                    ascent_start_time = currentTime;
                }
                break;
            }

            // ---- ASCENDING: 两阶段统一位置控制, 位置反馈轮询到位 ----
            case ASCENDING: {
                float currentPos = 0.0f;
                if (arm->readVerticalPosition(currentPos)) {
                    if (fabsf(currentPos - vertical_target_degrees) <= kPositionArrivedThreshDeg) {
                        Serial.printf("Ascent complete (pos=%.1f)\n", currentPos);
                        finishRun(currentTime);
                    }
                }
                // 超时保护
                if ((currentTime - ascent_start_time) > kAscentTimeoutMs) {
                    Serial.println("ERROR: Ascent timeout! Stopping.");
                    haltAndReset();
                }
                break;
            }
        }
    }
};
