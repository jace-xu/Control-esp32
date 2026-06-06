# ArmControl — 机械臂底层电机/夹爪控制

> 封装三个机械臂电机（共用 Serial2 总线，地址与底盘 1~4 错开）与一个夹爪舵机。
> 只管"怎么驱动"，不含任务时序/状态机（那是 *ArmSequence* 的职责）。
> [ ! ] 全程序只能创建一个实例（电机地址固定）。占位参数见 [doc/ARM_TUNING.md](../../doc/ARM_TUNING.md)。

## 管理的执行器
- 地址 5 角度电机：旋转对准，PID 速度控制（也可位置控制，PRE_CATCH 用）
- 地址 6 上下电机：升降，**位置控制**
- 地址 7 前后电机：进给对准，PID 速度控制
- 夹爪舵机：ESP32 LEDC PWM（50Hz，通道 4），grip/release 到固定角度

## 构造与析构
- `ArmControl()` — 取 `ControlSerial::get_instance()` 单例；初始化夹爪 LEDC PWM，上电默认 `release()`

## 停止
- `stop()` — 停三个电机（角度+上下+前后）并 `resetPID()`
- `stopServo()` — 只停角度+前后并 `resetPID()`，不动上下电机（视觉中断时防积分饱和）

## 视觉伺服（角度+前后 PID 速度闭环）
- `updateVisualServo(angleError, forwardError)`
    - 按 `millis()` 自算 dt（限幅 1ms~100ms）；误差在 `deadzone` 内则停转清零积分
    - 积分带 `integral_limit` 限幅，输出按 `max_rpm` 限幅
    - 角度+前后合并为一条长命令一次性下发（持锁 → clear → append×2 → send）
    - [ ! ] 误差单位是**像素**（树莓派输出），不是角度/毫米

## 位置控制（绝对角度）
- `setPosition(address, deg)` — 通用位置控制，`X_generate_set_position_command`(0xFD) **原样下发不取负**
    - 方向交给调用方：正角度→一个方向，负角度→反方向；速度 `kPosSpeed`
- `setVerticalPosition(deg)` — `setPosition(kVerticalAddr, deg)` 薄封装
- `setAnglePosition(deg)` — `setPosition(kAngleAddr, deg)` 薄封装（PRE_CATCH 预备摆位用）
- `readVerticalPosition(out)` — 问询 → 延迟 `kPositionReadDelayMs` 等回复 → 读响应；全程持锁为原子事务
    - 返回 true=成功；false=超时/校验失败，out 不变
    - [ ! ] 问询后必须留往返时间，否则在电机回复前就超时返回 false
- `manualVertical(rpm)` — 上下电机速度模式单发（备用，当前序列不再使用）

## 夹爪舵机
- `grip()` / `release()` — 转到 `kGripperClosedAngle` / `kGripperOpenAngle`
- 内部 `setGripperAngle(angle)` — 角度→脉宽→LEDC 占空比
- [ ! ] 普通 PWM 舵机无位置反馈，到位只能按时间估算（见 *ArmSequence* 的 settle 等待）

## 参数整定接口
- `setAnglePID(kp,ki,kd)` / `setForwardPID(kp,ki,kd)` — PID 增益
- `setDeadzone(dz)` / `setMaxRpm(max)` / `setIntegralLimit(limit)` — 死区/限速/抗饱和
- `resetPID()` — 清零积分、上次误差、更新时间

## 注意事项
- [ ! ] 只能创建一个实例
- [ ! ] 伺服长命令只含角度+前后；上下电机单独设位置
- [ ! ] 位置到位判定靠 *ArmSequence* 轮询 `readVerticalPosition()`，占总线
- [ ! ] 夹爪引脚/角度/脉宽、电机地址均为占位值，见 [doc/ARM_TUNING.md](../../doc/ARM_TUNING.md)
- 拷贝构造与赋值已禁用
