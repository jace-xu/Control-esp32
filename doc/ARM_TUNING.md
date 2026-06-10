# 机械臂参数整定清单

> 本文列出机械臂相关、**需按实际硬件 / 机械结构 / 现场调试修改**的参数。
> 标 **[必校]** 的不校准会直接导致动作错误或抓取失败；标 **[可调]** 的只影响节奏，不致命。


## 单位约定（重要）

- **视觉误差**（`angleError` / `forwardError`）单位是**像素**（树莓派图像偏差，原始范围 ±3000），不是角度或毫米。相关的死区、收敛阈值、PID 增益都在像素尺度上标定。
- **电机位置**（上下/角度电机）单位是**度**，相对上电零点的绝对角；1 圈 = 360°。
- **方向**由方向因子常量统一控制（见下「方向约定」），不要在多处散改符号。

---

## 1. 接线与通信（`src/main.cpp`）

| 常量 | 占位值 | 等级 | 说明 |
|------|--------|------|------|
| `kMotorRxPin` / `kMotorTxPin` | 16 / 17 | [必校] | 电机总线 Serial2 引脚，按实际接线 |
| `kVisionRxPin` / `kVisionTxPin` | 18 / 19 | [必校] | 视觉串口 Serial1 引脚（RX 接树莓派 TX） |
| `kMotorBaudRate` / `kVisionBaudRate` | 115200 | [可调] | 波特率，须与电机驱动器 / 树莓派一致 |
| `kFreshDataGraceMs` | 60ms | [可调] | 手柄数据宽限期，超时停机保护 |

---

## 2. 夹爪舵机（`lib/ArmControl/ArmControl.h`）

普通 PWM 舵机，**无位置反馈**，只能按时间估算闭合/张开耗时。

| 常量 | 占位值 | 等级 | 说明 / 整定建议 |
|------|--------|------|----------------|
| `kGripperPin` | 13 | [必校] | 舵机信号 GPIO，按实际接线；勿与底盘/串口冲突 |
| `kGripperPwmChannel` | 4 | [必校] | LEDC 通道，避开底盘可能占用的 0~3 |
| `kGripperClosedAngle` | 100° | [必校] | 夹紧角度，实测（夹住但不堵转） |
| `kGripperOpenAngle` | 180° | [必校] | 松开角度，实测（完全张开） |
| `kGripperMinPulseUs` / `kGripperMaxPulseUs` | 500 / 2500 | [必校] | 舵机脉宽范围，按舵机手册（多数 0.5~2.5ms 对应 0~180°） |

## 3. 电机地址（`lib/ArmControl/ArmControl.h`）

| 常量 | 值 | 等级 | 说明 |
|------|-----|------|------|
| `kAngleAddr` | 5 | [必校] | 角度电机（旋转对准 / PRE_CATCH 预备摆位） |
| `kVerticalAddr` | 6 | [必校] | 上下电机（升降，位置控制） |
| `kForwardAddr` | 7 | [必校] | 前后电机（进给对准） |

> 三个地址须与电机驱动器实际拨码一致，且与底盘电机（1~4）错开。

---

## 4. 方向约定（`lib/ArmSequence/ArmSequence.h`）

方向只在两个**因子常量**上表达一次，所有圈数/角度常量都乘它。实测方向反了，**只改因子符号**，不要去改各处目标值。

| 常量 | 占位值 | 等级 | 说明 |
|------|--------|------|------|
| `kVerticalDirection` | -1.0 | [必校] | 上下电机方向因子：负=下降，正=上升 |
| `kRotationDirection` | -1.0 | [必校] | 角度电机方向因子：正=逆时针，负=顺时针 |

> 底层 `setPosition(addr, deg)` 原样下发角度不取负：正角度→电机一个方向，负角度→反方向。
> 物理上哪个符号是"下降/正转"取决于电机接线与机械结构，须现场实测后定因子符号。

## 5. 位置控制目标与到位（`lib/ArmSequence/ArmSequence.h`）

圈数常量已乘 `kVerticalDirection`；目标绝对角度 = 圈数 × 360°。收臂归位目标为**绝对角（度）**，直接现场标定。

| 常量 | 占位值 | 等级 | 说明 / 整定建议 |
|------|--------|------|----------------|
| `kPreCatchAngleDeg` | 180°×方向 | [必校] | PRE_CATCH 角度电机预备摆位绝对角，实测预备位 |
| `kPreCatchForwardDeg` | 0° | [必校] | PRE_CATCH 前后电机预备摆位绝对角，实测预备位 |
| `kPreDescendRotations` | 0 圈 | [可调] | 对准期先降的圈数 |
| `kDescendTargetRotations` | 5 圈 | [必校] | 阶段1 下降到**夹取位**的圈数，按抓取高度定 |
| `kStowVerticalHomeDeg` | 0° | [必校] | 收臂步骤1 上下电机归位绝对角（0 位最高） |
| `kStowForwardHomeDeg` | 0° | [必校] | 收臂步骤1 前后电机归位绝对角（0 位收回） |
| `kStowAngleDeg` | 0° | [必校] | 收臂步骤2 角度电机归位绝对角 |
| `kStowForwardExtendDeg` | -430° | [必校] | 收臂步骤3 前后电机伸出绝对角（放料位）；方向反了改符号 |
| `kPositionArrivedThreshDeg` | 5° | [可调] | 位置到位判定阈值（电机角度）；太小可能判不到位 |
| `kPositionReadDelayMs`（ArmControl） | 2ms | [可调] | 位置问询后等电机回复的时间；太小读不到位置（返回 false） |

> 阶段1 收臂三步：① 上下→0 同时前后→0 ② 角度→0 ③ 前后伸出到 `kStowForwardExtendDeg` → 松爪放料 →（物料盘逻辑链，待补）。
> 阶段2（放置）目标圈数 `kPlaceDescendRotations` / `kPlaceAscendRotations` 在 `lib/PlaceSequence/PlaceSequence.h`。
> [ ! ] 收臂（STOW）轮询到位依赖角度(5)/上下(6)/前后(7)电机能读位置 → 其驱动器"响应"须开启。

## 6. 视觉伺服 PID（`src/main.cpp` 启动时下发；增益在像素尺度）

| 常量 | 占位值 | 等级 | 说明 |
|------|--------|------|------|
| `kArmAngleKp` / `kArmAngleKi` / `kArmAngleKd` | 30 / 0.5 / 5 | [必校] | 角度轴 PID 增益，按对准响应调 |
| `kArmForwardKp` / `kArmForwardKi` / `kArmForwardKd` | 30 / 0.5 / 5 | [必校] | 前后轴 PID 增益 |

ArmControl 内部相关（`setDeadzone`/`setMaxRpm`/`setIntegralLimit` 可改）：

| 成员 | 占位值 | 等级 | 说明 |
|------|--------|------|------|
| `deadzone` | 5 像素 | [可调] | 误差死区，\|误差\|<dz 该轴停转 |
| `max_rpm` | 50 rpm | [可调] | 伺服输出限幅 |
| `integral_limit` | 1000 | [可调] | 积分抗饱和限幅 |
| `kVerticalPosSpeed` / `kForwardPosSpeed` / `kAnglePosSpeed` | 800 / 120 / 15 rpm | [可调] | 位置移动速度（按电机区分）。**太慢会撞超时**：移动 N 圈耗时 ≈ N÷(speed/60) 秒，须 < 对应超时（`kDescentTimeoutMs` 等），否则没到位就被超时强停 |

> 对准方向若反：优先在视觉源头（树莓派）或 `updateVisualServo` 入口修正误差符号，
> 不要在输出 rpm 末端取反，也不要用负增益（会掩盖根因）。

## 7. 收敛判定与时序

收敛判定常量在 `lib/ArmControl/ArmControl.h`；时序超时在 `lib/ArmSequence/ArmSequence.h`。

> [ ! ] 当前为**手动确认模式**：`ArmControl::isAligned()` 恒返回 true，收敛计数块已注释。
> 即第二次按 A 即视为"已对准"直接下降，不等自动收敛。下表收敛阈值**当前不生效**，
> 若要恢复自动收敛：取消 `updateAlign` 里收敛计数块注释，并把 `isAligned()` 改回
> `return converge_count >= kConvergeFramesNeeded;`。

| 常量 | 占位值 | 等级 | 说明 |
|------|--------|------|------|
| `kConvergeFramesNeeded`（ArmControl） | 10 | [可调] | 连续入阈帧数才算对准（50Hz≈0.2s）。当前不生效 |
| `kConvergeAngleThresh`（ArmControl） | 8 像素 | [必校] | 角度轴收敛判据。当前不生效 |
| `kConvergeForwardThresh`（ArmControl） | 8 像素 | [必校] | 前后轴收敛判据。当前不生效 |
| `kGripSettleMs`（ArmSequence） | 800ms | [可调] | 阶段1 舵机夹紧到位等待（无反馈，按实测闭合耗时，典型 150~400ms） |
| `kPreCatchTimeoutMs`（ArmSequence） | 30s | [可调] | PRE_CATCH 等按 A 的超时 |
| `kPreAlignTimeoutMs`（ArmSequence） | 30s | [可调] | PRE_ALIGN 等确认键的超时 |
| `kDescentTimeoutMs` / `kAscentTimeoutMs`（ArmSequence） | 15s / 12s | [可调] | 下降/收臂归位超时保护（位置反馈失败/卡死兜底） |
| `kStartRetryIntervalMs` / `kStartMaxRetries`（VisionStreamHandshake） | 300ms / 10 | [可调] | 视觉 START 握手重传间隔/次数 |

> 阶段2（放置）的等待 `kGripDelayMs` 等时序在 `lib/PlaceSequence/PlaceSequence.h`。

---

## 上车前必校清单（最小集）

1. 接线引脚（第 1 节）与电机地址（第 3 节）
2. 夹爪夹紧/松开角度（第 2 节）
3. 两个方向因子 `kVerticalDirection` / `kRotationDirection`（第 4 节）—— 先确认下降/正转方向
4. 夹取位圈数 `kDescendTargetRotations`（第 5 节）、放置位圈数（PlaceSequence）
5. 视觉对准 PID 与收敛阈值（第 6、7 节）—— 确认对准方向不反
