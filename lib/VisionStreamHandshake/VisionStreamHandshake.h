#pragma once
#include <Arduino.h>
#include <VisionSerial.h>

/**
 * @brief 视觉串流握手会话 (START/STOP + 超时重传)
 * @note  树莓派仅在 ESP32 请求后才发误差数据。本类管理"串流会话状态":
 *        是否已请求 / 是否收到首帧确认 / START 是否需要超时重传。
 * @note  分层: 本类是会话状态机, 持有 VisionSerial 指针 (无状态协议收发层),
 *        通过它真正发 START1/START2/STOP。VisionSerial 保持无状态可复用,
 *        会话状态只集中在本类, 不污染传输层。
 * @note  依赖注入: 构造传入已初始化的 VisionSerial 指针, 本类不拥有其生命周期。
 * @note  == Version 1.0.0 ==
 */
class VisionStreamHandshake {
private:
    // START 命令可能被干扰丢失, 在收到首帧数据前定时重发
    static constexpr uint32_t kStartRetryIntervalMs = 300;  // 未收到数据则每 300ms 重发 START
    static constexpr uint8_t kStartMaxRetries = 10;         // START 最大重传次数 (约 3 秒)

    VisionSerial* vision = nullptr;     // 视觉数据串口 (无状态协议收发层, 不拥有生命周期)

    bool streaming = false;             // 当前是否已请求树莓派发送误差数据
    bool confirmed = false;             // 是否已收到首帧有效数据 (确认对接成功)
    uint32_t last_start_sent_time = 0;  // 上次发送 START 命令的时间戳 (用于超时重传)
    uint8_t start_retry_count = 0;      // START 已重传次数
    bool start_failed_warned = false;   // 重传耗尽的失败警告是否已打印过

    // 本次串流请求的误差类型与颜色 (重传时需重发同一条命令)
    uint8_t error_type = 1;             // 1=误差1, 2=误差2(按颜色)
    uint8_t color = 0;                  // 误差2 的目标颜色(1=绿/2=蓝/3=红); 误差1 时为 0

    /// @brief 按当前 error_type/color 发送对应的 START 命令
    void sendStartCommand() {
        if (error_type == 2) {
            vision->requestStart2(color);
        } else {
            vision->requestStart1();
        }
    }

public:
    // 禁用拷贝和赋值
    VisionStreamHandshake(const VisionStreamHandshake&) = delete;
    VisionStreamHandshake& operator=(const VisionStreamHandshake&) = delete;

    /**
     * @brief 构造视觉串流握手会话
     * @param vision 已初始化的 VisionSerial 指针 (协议收发层)
     */
    explicit VisionStreamHandshake(VisionSerial* vision) : vision(vision) {}

    /// @brief 当前是否已请求串流 (供 ArmSequence 判断是否进伺服分支)
    bool isStreaming() const { return streaming; }

    /// @brief 是否已收到首帧有效数据 (确认对接成功)
    bool isConfirmed() const { return confirmed; }

    /// @brief 本次串流请求的误差类型 (1/2), 供帧类型校验
    uint8_t expectedErrorType() const { return error_type; }

    /**
     * @brief 请求树莓派开始发送视觉误差数据 (幂等: 已在串流则不重发)
     * @param errorType   误差类型: 1=误差1, 2=误差2(按颜色)
     * @param targetColor 误差2 的目标颜色(1=绿/2=蓝/3=红); 误差1 传 0
     * @param currentTime 当前 millis() 时间戳, 用于初始化重传计时
     */
    void start(uint8_t errorType, uint8_t targetColor, uint32_t currentTime) {
        if (!streaming) {
            error_type = errorType;
            color = targetColor;
            streaming = true;
            confirmed = false;          // 等待首帧数据确认对接
            last_start_sent_time = currentTime;
            start_retry_count = 0;
            start_failed_warned = false;
            sendStartCommand();
        }
    }

    /// @brief 通知树莓派停止发送视觉误差数据 (幂等: 未串流则不重发)
    void stop() {
        if (streaming) {
            vision->requestStop();
            streaming = false;
            confirmed = false;
        }
    }

    /// @brief 标记已收到首帧匹配数据 (停止 START 重传); 供 runVisualServo 收到匹配帧时调用
    void confirm() { confirmed = true; }

    /**
     * @brief START 命令超时重传检查
     * @param currentTime 当前 millis() 时间戳
     * @details 已请求串流但尚未收到首帧数据时, 每隔 kStartRetryIntervalMs 重发一次
     *          START, 防止握手帧被干扰丢失导致树莓派始终不发数据。收到首帧后
     *          (confirmed=true) 停止重传; 超过最大次数后放弃并打印一次警告。
     * @note  边降边对准模式: 重传耗尽不中止序列, 机械臂继续按时间序列动作 (可能盲抓),
     *        仅打印警告提示握手失败。
     */
    void retryIfNeeded(uint32_t currentTime) {
        if (!streaming || confirmed) {
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
};
