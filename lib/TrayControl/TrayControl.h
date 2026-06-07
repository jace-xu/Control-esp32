#pragma once
#include <ControlSerial.h>

/**
 * @brief 物料盘控制类 (骨架, 动作待补)
 * @note  物料盘是独立硬件子系统, 有自己的电机/舵机, 负责存料 / 转位 / 出料。
 * @note  与底盘(1~4)、机械臂(5~7)共用 Serial2 总线; 物料盘电机地址须 >=8 避开冲突。
 * @note  [ ! ] 当前仅占位空类: 构造取 ControlSerial 单例; 动作方法等硬件细节
 *        (地址 / 槽位数 / 转位 / 出料方式) 确定后再补。
 * @note  使用范式同 BottomControl / ArmControl (按地址直发命令)。
 * @note  == Version 0.1.0 (skeleton) ==
 */
class TrayControl {
private:
    // ---- 物料盘电机/舵机总线地址 (待定, >=8 避开底盘1~4、机械臂5~7) ----
    // [ ! ] 占位: 等硬件确定后填写实际地址常量, 例如:
    // static constexpr int kTurntableAddr = 8;   // 转盘电机
    // static constexpr int kPusherAddr    = 9;   // 推料电机/舵机

    ControlSerial* control_serial = nullptr;  // 共用的 Serial2 命令通道 (单例)

public:
    // 禁用拷贝和赋值
    TrayControl(const TrayControl&) = delete;
    TrayControl& operator=(const TrayControl&) = delete;

    /**
     * @brief 创建物料盘控制对象
     * @note  通过 ControlSerial::get_instance() 取得唯一串口实例 (与底盘/机械臂共用)。
     */
    TrayControl() {
        this->control_serial = &(ControlSerial::get_instance());
    }

    // ========================================================================
    // 动作接口 (待补): 等物料盘硬件细节确定后, 在此添加如：
    //   - store(color)    : 接收机械臂放入的物料, 存入槽位
    //   - dispense(color) : 把指定颜色物料送到取料位供机械臂夹取
    //   - update(t)       : 物料盘自身的状态机/到位推进 (若动作非阻塞)
    //   - isBusy()        : 物料盘是否正在执行动作
    //   - stop()          : 急停所有物料盘电机
    // ========================================================================
};
