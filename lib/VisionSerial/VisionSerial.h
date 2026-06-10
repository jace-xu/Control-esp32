#pragma once
#include <Arduino.h>

/**
 * @brief 误差2 的目标物块颜色 (供 main / ArmSequence 选择抓取颜色)
 */
enum BlockColor : uint8_t {
    kColorNone = 0,   // 无 (误差1 不使用颜色)
    kColorGreen = 1,  // 绿色
    kColorBlue = 2,   // 蓝色
    kColorRed = 3,    // 红色
};

/**
 * @brief Vision error data structure
 */
struct VisionError {
    bool valid = false;           // True if data was successfully parsed
    float angleError = 0.0f;      // Angle axis error (±3000)
    float forwardError = 0.0f;    // Forward axis error (±3000)
    uint32_t timestampMs = 0;     // Timestamp when data was received
    uint8_t id1 = 0;              // 误差类型: 1=误差1, 2=误差2
    uint8_t id2 = 0;              // 物块颜色: 1=绿, 2=蓝, 3=红
};

/**
 * @brief Vision serial communication class
 * @note Receives error data from Raspberry Pi via UART
 * @note 误差包协议: 0xAA + id1 + id2 + Data1(int16 LE) + Data2(int16 LE) + 校验(前7字节异或)
 * @note Total 8 bytes per packet
 * @note == Version 3.0.0 ==
 */
class VisionSerial {
private:
    HardwareSerial* serial = nullptr;
    VisionError lastError;

    static constexpr uint8_t kFrameHeader = 0xAA;
    static constexpr size_t kPacketSize = 8;
    static constexpr uint32_t kDataTimeoutMs = 500;

    // [调试] 原始字节嗅探: 把视觉串口收到的原始字节打印到 USB 串口, 用于排查链路。
    //   - 完全没打印      → 一个字节都没收到: 查接线(RX=GPIO18接树莓派TX)/共地/树莓派程序/波特率
    //   - 打印但无 0xAA   → 收到数据但帧头不对: 协议/波特率不匹配
    //   - 有 0xAA 仍无伺服 → 帧头对但校验或 id1 类型不匹配 (看 parsePacket 丢弃)
    // 调通后改为 false 关闭。
    static constexpr bool kDebugRawBytes = false;

    // ESP32 → 树莓派 命令协议 (与误差数据方向相反)
    // 包格式: 0xBB + CMD + PARAM + 校验(0xBB ^ CMD ^ PARAM), 共 4 字节
    static constexpr uint8_t kCmdHeader = 0xBB;   // 命令包头 (区别于误差包 0xAA)
    static constexpr uint8_t kCmdStop = 0x00;     // 命令: 停止发送误差 (PARAM=0)
    static constexpr uint8_t kCmdStart1 = 0x01;   // 命令: 开始发送误差1 (PARAM=0)
    static constexpr uint8_t kCmdStart2 = 0x02;   // 命令: 开始发送误差2 (PARAM=物块颜色)
    // 误差2 的颜色取值见文件顶部 BlockColor 枚举

    uint8_t buffer[kPacketSize];
    size_t bufferIndex = 0;
    bool foundHeader = false;

public:
    // Disabled copying and assignment
    VisionSerial(const VisionSerial&) = delete;
    VisionSerial& operator=(const VisionSerial&) = delete;

    /**
     * @brief Create vision serial object
     * @param baudRate Baud rate (typically 115200)
     * @param rxPin RX pin (GPIO number)
     * @param txPin TX pin (GPIO number)
     */
    VisionSerial(int baudRate, int rxPin, int txPin) {
        this->serial = &Serial1;
        this->serial->begin(baudRate, SERIAL_8N1, rxPin, txPin);
        delay(100);
    }

    /**
     * @brief Read and parse vision error data (non-blocking)
     * @return VisionError structure with valid flag and error values
     * @note Call this frequently in main loop
     */
    VisionError read() {
        // Drain the ENTIRE RX buffer every call, keeping only the freshest
        // valid packet. Returning on the first packet would leave the rest of
        // the buffer backlogged: on ESP32 the consumer (~50Hz) cannot outrun a
        // ~50Hz producer one-packet-at-a-time, so stale data would accumulate
        // and the visual servo would track a target hundreds of ms old.
        bool gotNewPacket = false;

        while (serial->available() > 0) {
            uint8_t byte = serial->read();

            // [调试] 打印每个收到的原始字节 (调通后把 kDebugRawBytes 置 false)
            if (kDebugRawBytes) {
                Serial.printf("VRX %02X\n", byte);
            }

            // State machine: looking for header
            if (!foundHeader) {
                if (byte == kFrameHeader) {
                    foundHeader = true;
                    bufferIndex = 0;
                    buffer[bufferIndex++] = byte;
                }
                continue;
            }

            // Collecting packet data
            buffer[bufferIndex++] = byte;

            // Complete packet received
            if (bufferIndex >= kPacketSize) {
                VisionError error = parsePacket(buffer);

                // Reset state machine
                foundHeader = false;
                bufferIndex = 0;

                if (error.valid) {
                    // Overwrite with the latest valid frame, but keep draining
                    // the buffer so the next call starts empty.
                    lastError = error;
                    lastError.timestampMs = millis();
                    gotNewPacket = true;
                }
            }
        }

        if (gotNewPacket) {
            return lastError;
        }

        // Check if last data is too old
        if (lastError.valid && (millis() - lastError.timestampMs) > kDataTimeoutMs) {
            lastError.valid = false;
        }

        return lastError;
    }

    /**
     * @brief 通知树莓派开始发送 误差1
     * @note  发送命令包 0xBB + 0x01 + 0x00 + 校验。
     */
    void requestStart1() {
        sendCommand(kCmdStart1, 0x00);
    }

    /**
     * @brief 通知树莓派开始发送 误差2 (按物块颜色)
     * @param color 目标颜色 (kColorGreen / kColorBlue / kColorRed)
     * @note  发送命令包 0xBB + 0x02 + color + 校验。
     */
    void requestStart2(uint8_t color) {
        sendCommand(kCmdStart2, color);
    }

    /**
     * @brief 通知树莓派停止发送视觉误差数据
     * @note  发送命令包 0xBB + 0x00 + 0x00 + 校验。在夹取完成 / 中止 / 安全停止时调用。
     */
    void requestStop() {
        sendCommand(kCmdStop, 0x00);
    }

private:
    /**
     * @brief 向树莓派发送一个命令包
     * @param cmd   命令字节 (kCmdStart1 / kCmdStart2 / kCmdStop)
     * @param param 命令参数 (误差2 为颜色, 其余为 0)
     * @note  包格式: 0xBB + CMD + PARAM + 校验(0xBB ^ CMD ^ PARAM), 共 4 字节
     */
    void sendCommand(uint8_t cmd, uint8_t param) {
        uint8_t packet[4] = {
            kCmdHeader,
            cmd,
            param,
            static_cast<uint8_t>(kCmdHeader ^ cmd ^ param),
        };
        serial->write(packet, sizeof(packet));
    }
    /**
     * @brief Parse binary packet into VisionError
     * @param packet 8-byte packet buffer
     * @return VisionError with valid=true if parsing succeeded
     * @note 布局: [0]=0xAA [1]=id1 [2]=id2 [3..4]=data1(LE) [5..6]=data2(LE) [7]=校验
     *       校验 = packet[0..6] 的逐字节异或
     */
    VisionError parsePacket(const uint8_t* packet) {
        VisionError error;
        error.valid = false;

        // Verify header
        if (packet[0] != kFrameHeader) {
            return error;
        }

        // 校验: 前 7 字节异或应等于第 8 字节
        uint8_t checksum = 0;
        for (size_t i = 0; i < kPacketSize - 1; ++i) {
            checksum ^= packet[i];
        }
        if (checksum != packet[kPacketSize - 1]) {
            return error;  // 校验失败, 丢弃
        }

        // Extract IDs
        error.id1 = packet[1];  // 误差类型 (1/2)
        error.id2 = packet[2];  // 物块颜色 (1=绿/2=蓝/3=红)

        // Extract Data1 (angle error) - int16 little-endian
        int16_t data1_raw = static_cast<int16_t>(packet[3] | (packet[4] << 8));
        error.angleError = static_cast<float>(data1_raw);

        // Extract Data2 (forward error) - int16 little-endian
        int16_t data2_raw = static_cast<int16_t>(packet[5] | (packet[6] << 8));
        error.forwardError = static_cast<float>(data2_raw);

        // Validate range (±3000)
        if (fabsf(error.angleError) > 3000.0f || fabsf(error.forwardError) > 3000.0f) {
            return error;  // Out of range, invalid
        }

        error.valid = true;
        return error;
    }
};
