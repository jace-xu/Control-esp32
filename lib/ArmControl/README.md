# 2026.06.04

## 这是机械臂底层控制类，封装了三个机械臂电机的伺服与手动控制
- 管理三个电机（共用 Serial2 总线，地址与底盘电机 1~4 错开）
    - 地址 5：角度电机（旋转对准）
    - 地址 6：上下电机（升降）
    - 地址 7：前后电机（进给对准）
- [ ! ] 全程序只能创建一个 ArmControl 对象（三个电机地址固定）
- 不负责视觉数据的获取与任务编排

## 构造与析构
- `ArmControl()`
    - 通过 `ControlSerial::get_instance()` 取得唯一串口实例
    - 不再创建 *Motor* 对象，直接用 `ControlSerial` 按地址（5/6/7）发命令
- `~ArmControl()`
    - 空实现（不持有需要释放的资源）

## 停止控制
- `stop()`
    - 停止全部三个电机（角度 + 上下 + 前后）
    - 同时调用 `resetPID()` 复位 PID 状态，保证下次伺服从干净状态开始
- `stopServo()`
    - 只停角度 + 前后电机，不动上下电机
    - 同时 `resetPID()`，防止视觉数据中断恢复后积分饱和（windup）窜动
    - 上下电机是速度开环控制，不受 `resetPID()` 影响

## 视觉伺服
- `updateVisualServo(angleError, forwardError)`
    - 自动按 `millis()` 计算时间间隔 dt（限幅 1ms ~ 100ms，防止除零与异常）
    - 误差在死区（`deadzone`）内时电机停转，并清零该轴积分项
    - 积分项带限幅（`integral_limit`），防止积分饱和
    - 输出按 `max_rpm` 限幅
    - 角度 + 前后 + 上下 三条命令合并为一条长命令一次性下发
        - 全程持锁 → `clear_long_command()` → append 角度 → append 前后 → append 上下 → `send_long_command()`
        - **[!]后续更改** 上下电机不参与 PID，用 `vertical_rpm` 缓存的当前指令一并重发
        

## 上下电机（目前是速度模式）
- `manualVertical(rpm)`
    - 速度模式控制上下电机，正值上升、负值下降
    - 既更新 `vertical_rpm` 缓存，也立即单独下发一次（状态切换瞬间即时生效，不必等下一伺服帧）
    - [ ! ] 缓存值会被伺服帧每帧重发，所以停机时必须清零（`stop()` 已处理）
- `setVerticalPosition(position)`
    - 预留接口（位置控制协议 0xFD 尚未实现），当前为空实现
    - [ ! ] 升降目前全部用时间控制，此函数留待将来实现位置反馈后填充

## 参数整定
- `setAnglePID(kp, ki, kd)` — 设置角度轴 PID 增益
- `setForwardPID(kp, ki, kd)` — 设置前后轴 PID 增益
- `setDeadzone(dz)` — 设置误差死区阈值（|误差| < dz 时电机停转）
- `setMaxRpm(max)` — 设置电机最大转速限制
- `setIntegralLimit(limit)` — 设置积分项限幅（抗饱和）
- `resetPID()`
    - 清零两个轴的积分项、上一次误差、上次更新时间
    - 开始一次新的伺服会话时调用

## 注意事项
- [ ! ] 只能创建一个实例，重复创建会导致地址冲突
- [ ! ] 三电机速度命令暂用 `Emm_generate_set_rotate_speed_command`，将来改用 `X_generate_set_rotate_speed_command`（该指令尚未实现，代码中已留 TODO）
- [ ! ] 长命令打包要求电机驱动固件支持批量帧（已确认支持）
- 拷贝构造与赋值已禁用
