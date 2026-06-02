#pragma once
#include <Arduino.h>

/**
 * @brief Vision error data structure
 */
struct VisionError {
    bool valid = false;           // True if data was successfully parsed
    float angleError = 0.0f;      // Angle axis error (±3000)
    float forwardError = 0.0f;    // Forward axis error (±3000)
    uint32_t timestampMs = 0;     // Timestamp when data was received
    uint8_t id = 0;               // Packet ID
};

/**
 * @brief Vision serial communication class
 * @note Receives error data from Raspberry Pi via UART
 * @note Protocol: 0xAA + ID(uint8) + Data1(int16 LE) + Data2(int16 LE)
 * @note Total 6 bytes per packet
 * @note == Version 2.0.0 ==
 */
class VisionSerial {
private:
    HardwareSerial* serial = nullptr;
    VisionError lastError;

    static constexpr uint8_t kFrameHeader = 0xAA;
    static constexpr size_t kPacketSize = 6;
    static constexpr uint32_t kDataTimeoutMs = 500;

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
        // Read available bytes
        while (serial->available() > 0) {
            uint8_t byte = serial->read();

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
                    lastError = error;
                    lastError.timestampMs = millis();
                    return lastError;
                }
            }
        }

        // Check if last data is too old
        if (lastError.valid && (millis() - lastError.timestampMs) > kDataTimeoutMs) {
            lastError.valid = false;
        }

        return lastError;
    }

private:
    /**
     * @brief Parse binary packet into VisionError
     * @param packet 6-byte packet buffer
     * @return VisionError with valid=true if parsing succeeded
     */
    VisionError parsePacket(const uint8_t* packet) {
        VisionError error;
        error.valid = false;

        // Verify header
        if (packet[0] != kFrameHeader) {
            return error;
        }

        // Extract ID
        error.id = packet[1];

        // Extract Data1 (angle error) - int16 little-endian
        int16_t data1_raw = static_cast<int16_t>(packet[2] | (packet[3] << 8));
        error.angleError = static_cast<float>(data1_raw);

        // Extract Data2 (forward error) - int16 little-endian
        int16_t data2_raw = static_cast<int16_t>(packet[4] | (packet[5] << 8));
        error.forwardError = static_cast<float>(data2_raw);

        // Validate range (±3000)
        if (fabsf(error.angleError) > 3000.0f || fabsf(error.forwardError) > 3000.0f) {
            return error;  // Out of range, invalid
        }

        error.valid = true;
        return error;
    }
};
