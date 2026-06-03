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
    // 任务模式: 决定本次/本轮序列的行为
    // ========================================================================
    enum SequenceMode {
        MODE_NONE,    // 无任务
        MODE_RECORD,  // 阶段1 (A键): 请求误差1, 握手后记录一个颜色入队, 跑完整序列
        MODE_CONSUME, // 阶段2 (X键): 请求误差2, 按队列 FIFO 逐个颜色跑序列直到队空
    };

    // 颜色队列容量 (阶段1 最多可记录多少个待抓物料)
    static constexpr uint8_t kColorQueueCapacity = 16;

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

    // 视觉串流握手重传参数: START 命令可能被干扰丢失, 在收到首帧数据前定时重发
    static constexpr uint32_t kStartRetryIntervalMs = 300;  // 未收到数据则每 300ms 重发 START
    static constexpr uint8_t kStartMaxRetries = 10;         // START 最大重传次数 (约 3 秒)

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

    uint8_t color_queue[kColorQueueCapacity]; // 颜色环形队列
    uint8_t queue_head = 0;                 // 队首索引 (出队位置)
    uint8_t queue_tail = 0;                 // 队尾索引 (入队位置)
    uint8_t queue_count = 0;                // 队列当前元素个数

    // 触发边沿检测: A 键(记录) 与 X 键(消费) 各自检测上升沿
    bool prev_a_pressed = false;            // 上一帧 A 键是否按住
    bool prev_x_pressed = false;            // 上一帧 X 键是否按住

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
     * @details A / X 上升沿在空闲时各自触发对应模式; 序列自锁运行 (松开不影响)。
     *          阶段1 每按一次 A 记录一个颜色; 阶段2 按一次 X 自动跑完队列里所有颜色。
     *          运行期间按 B 立即停止。
     */
    void update(bool aPressed, bool xPressed, bool abortPressed) {
        const uint32_t currentTime = millis();

        // 中止: 序列运行期间按下 B 键 → 立即停止所有动作并复位为 IDLE
        if (active && abortPressed) {
            Serial.println("Sequence aborted by user");
            stop();
            prev_a_pressed = aPressed;
            prev_x_pressed = xPressed;
            return;
        }

        // 启动 (仅空闲时响应上升沿): A → 阶段1记色, X → 阶段2消费队列
        if (!active) {
            if (aPressed && !prev_a_pressed) {
                startRecordSequence(currentTime);
            } else if (xPressed && !prev_x_pressed) {
                startConsumeStage(currentTime);
            }
        }
        prev_a_pressed = aPressed;
        prev_x_pressed = xPressed;

        // 自锁运行: 序列一旦启动, 不再依赖按键是否按住
        if (active) {
            if (vision_streaming) {
                retryStartIfNeeded(currentTime);  // 未收到数据则超时重传 START
                runVisualServo(currentTime);
            } else {
                arm->stopServo();
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
        stopVisionStream();  // 通知树莓派停止发送误差数据
        if (arm != nullptr) {
            arm->stop();
        }
        state = IDLE;
        active = false;
        mode = MODE_NONE;
    }

    /// @brief 清空颜色队列 (供 main 在需要时手动复位任务)
    void clearColorQueue() {
        queue_head = 0;
        queue_tail = 0;
        queue_count = 0;
    }

private:
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
     * @brief 启动一次升降序列 (两阶段共用): 请求对应误差串流 + 进入 DESCENDING
     * @param currentTime 当前 millis() 时间戳
     * @param errorType   1=误差1, 2=误差2
     * @param color       误差2 的目标颜色; 误差1 传 kColorNone
     */
    void beginVerticalSequence(uint32_t currentTime, uint8_t errorType, uint8_t color) {
        active = true;
        startVisionStream(currentTime, errorType, color);  // 请求串流 (带超时重传)

        Serial.println("Starting descent");
        state = DESCENDING;
        descent_start_time = currentTime;

        // 新一次序列开始时复位 PID 状态, 清除上一次遗留的积分值
        arm->resetPID();

        // TODO: 位置控制协议 (0xFD) 尚未实现, 目前使用速度模式开环控制
        // 将来替换为: arm->setVerticalPosition(kDescendTargetRotations * 360);
        arm->manualVertical(-kVerticalMoveSpeed);  // 负值 = 下降
    }

    /**
     * @brief 本轮升降序列正常跑完后的推进逻辑
     * @param currentTime 当前 millis() 时间戳
     * @details 阶段2(消费): 队列还有颜色 → 自动启动下一个颜色的序列; 队空 → 结束。
     *          阶段1(记录): 一轮记一个颜色, 跑完即结束, 等待下次按 A。
     */
    void finishRun(uint32_t currentTime) {
        // 先停掉本轮的串流与电机 (回到静止)
        stopVisionStream();
        arm->manualVertical(0.0f);
        arm->stopServo();

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
        state = IDLE;
        active = false;
        mode = MODE_NONE;
    }

    /**
     * @brief 异常停止当前序列 (超时保护): 停电机并解除自锁
     * @note  与 finishRun() 区别: 此处用于超时等异常, 不推进队列, 直接结束本次任务。
     */
    void finishSequence() {
        stopVisionStream();
        arm->manualVertical(0.0f);
        arm->stopServo();
        state = IDLE;
        active = false;
        mode = MODE_NONE;
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

        if (visionError.valid) {
            // 数据有效: 确认握手成功 (停止 START 重传), 清除超时警告, 执行 PID 伺服
            stream_confirmed = true;
            timeout_warned = false;

            // 阶段1: 握手成功后记录本轮第一个颜色入队 (每轮只记一个)
            if (mode == MODE_RECORD && !color_recorded_this_run) {
                enqueueColor(visionError.id2);
                color_recorded_this_run = true;
            }

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
     *        DESCENDING → GRIPPING → ASCENDING → IDLE(结束)
     * @param currentTime 当前 millis() 时间戳
     * @note  启动逻辑在 beginVerticalSequence() (A/X 上升沿触发), 故运行期不会停在 IDLE。
     */
    void runVerticalSequence(uint32_t currentTime) {
        switch (state) {
            case IDLE:
                // 自锁运行期不应停在 IDLE (启动即进 DESCENDING); 防御性留空
                break;

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
                    finishSequence();
                }
                break;
            }

            case GRIPPING: {
                // 在底部等待夹取完成 (默认 5 秒)
                if ((currentTime - grip_start_time) >= kGripDelayMs) {
                    Serial.println("Starting ascent to 1 rotation");
                    state = ASCENDING;
                    ascent_start_time = currentTime;

                    // 夹取完成, 回升阶段不再需要视觉: 通知树莓派停止发送误差,
                    // 并停伺服, 避免回升期间残留误差驱动角度/前后电机乱动
                    stopVisionStream();
                    arm->stopServo();

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
                    finishRun(currentTime);  // 正常跑完: 阶段2自动推进下一颜色, 否则结束
                }

                // 回升超时保护: 超过 12 秒强制停止
                else if ((currentTime - ascent_start_time) > kAscentTimeoutMs) {
                    Serial.println("ERROR: Ascent timeout! Stopping.");
                    finishSequence();
                }
                break;
            }
        }
    }
};
