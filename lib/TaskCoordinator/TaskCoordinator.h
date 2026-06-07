#pragma once
#include <Arduino.h>
#include <ArmSequence.h>
#include <TrayControl.h>

/**
 * @brief 上层任务协调器 (骨架, 当前为透传层)
 * @note  未来承载"阶段调度 + 机械臂↔物料盘协同": 阶段1 机械臂夹取→收臂→放入物料盘;
 *        中间物料盘存储; 出料阶段物料盘送出→机械臂夹取→放置。
 * @note  依赖注入: 构造传入 ArmSequence 与 TrayControl 指针, 本类不拥有其生命周期。
 * @note  [ ! ] 当前实现仅把 update 透传给 ArmSequence (保持现有行为不变);
 *        TrayControl 动作与阶段调度逻辑等物料盘硬件细节确定后在此填充。
 * @note  == Version 0.1.0 (skeleton, passthrough) ==
 */
class TaskCoordinator {
private:
    ArmSequence* arm_seq = nullptr;   // 机械臂任务序列 (夹取/放置动作)
    TrayControl* tray = nullptr;      // 物料盘 (存料/出料, 动作待补)

public:
    // 禁用拷贝和赋值
    TaskCoordinator(const TaskCoordinator&) = delete;
    TaskCoordinator& operator=(const TaskCoordinator&) = delete;

    /**
     * @brief 构造任务协调器
     * @param armSeq 已初始化的机械臂任务序列指针
     * @param tray   已初始化的物料盘控制指针
     */
    TaskCoordinator(ArmSequence* armSeq, TrayControl* tray)
        : arm_seq(armSeq), tray(tray) {}

    /**
     * @brief 每帧主入口 (按键转发 + 未来阶段调度)
     * @param aPressed     A 键
     * @param xPressed     X 键
     * @param abortPressed B 键
     * @param l1Pressed    L1 键
     * @note  当前: 直接透传给 ArmSequence::update (行为与改动前一致)。
     *        未来: 在此按阶段插入 TrayControl 的 store/dispense 调度, 并与机械臂动作协同。
     */
    void update(bool aPressed, bool xPressed, bool abortPressed, bool l1Pressed) {
        if (arm_seq != nullptr) {
            arm_seq->update(aPressed, xPressed, abortPressed, l1Pressed);
        }
        // [ ! ] 预留: 物料盘阶段调度 (等 TrayControl 动作就位)
        //   - 阶段1 机械臂放料到位后 → tray->store(color)
        //   - 出料阶段 → tray->dispense(color), 到位后交给机械臂夹取
        //   - if (tray != nullptr) tray->update(millis());
    }

    /// @brief 是否有动作正在运行 (供 main 安全判断)
    /// @note  当前仅查机械臂; 未来或并入物料盘忙状态 (|| tray->isBusy())。
    bool isActive() const {
        return arm_seq != nullptr && arm_seq->isActive();
    }

    /// @brief 立即停止 (中止/急停时调用)
    /// @note  当前转发机械臂停止; 未来加 tray->stop()。
    void stop() {
        if (arm_seq != nullptr) {
            arm_seq->stop();
        }
        // [ ! ] 预留: if (tray != nullptr) tray->stop();
    }
};
