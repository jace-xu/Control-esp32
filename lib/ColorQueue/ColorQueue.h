#pragma once
#include <Arduino.h>

/**
 * @brief 颜色 FIFO 环形缓冲 (物料颜色待抓队列)
 * @note  阶段1 (记录) 每轮入队一个物料颜色; 阶段2 (消费) 按 FIFO 逐个出队。
 * @note  纯数据结构, 与机械臂状态机解耦; 供 ArmSequence / 未来上层协调器复用。
 * @note  容量固定 kCapacity; 非线程安全 (仅主循环单线程使用)。
 * @note  == Version 1.0.0 ==
 */
class ColorQueue {
public:
    /// @brief 队列容量 (最多可记录多少个待抓物料)
    static constexpr uint8_t kCapacity = 16;

    /// @brief 当前队列中元素个数
    uint8_t count() const { return count_; }

    /// @brief 队列是否为空
    bool isEmpty() const { return count_ == 0; }

    /// @brief 队列是否已满
    bool isFull() const { return count_ >= kCapacity; }

    /**
     * @brief 颜色入队 (队尾)
     * @param color 物料颜色 (1=绿/2=蓝/3=红)
     * @return true=入队成功; false=队满丢弃
     */
    bool enqueue(uint8_t color) {
        if (isFull()) {
            return false;
        }
        buffer_[tail_] = color;
        tail_ = (tail_ + 1) % kCapacity;
        count_++;
        return true;
    }

    /**
     * @brief 颜色出队 (取队首)
     * @param color [out] 取出的颜色; 队空时不变
     * @return true=出队成功; false=队空
     */
    bool dequeue(uint8_t& color) {
        if (isEmpty()) {
            return false;
        }
        color = buffer_[head_];
        head_ = (head_ + 1) % kCapacity;
        count_--;
        return true;
    }

    /**
     * @brief 撤销最近一次入队的元素 (回退队尾指针)
     * @return true=成功丢弃; false=队空
     * @note  供阶段1 中止回滚用: 本轮至多入队一个且未出队, 故队尾必为本轮所记。
     */
    bool dropLast() {
        if (isEmpty()) {
            return false;
        }
        tail_ = (tail_ + kCapacity - 1) % kCapacity;
        count_--;
        return true;
    }

    /// @brief 清空队列
    void clear() {
        head_ = 0;
        tail_ = 0;
        count_ = 0;
    }

private:
    uint8_t buffer_[kCapacity] = {0};  // 环形缓冲
    uint8_t head_ = 0;                 // 队首索引 (出队位置)
    uint8_t tail_ = 0;                 // 队尾索引 (入队位置)
    uint8_t count_ = 0;                // 当前元素个数
};
