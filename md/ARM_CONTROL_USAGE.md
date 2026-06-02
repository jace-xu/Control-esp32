# 机械臂视觉伺服控制 - 使用说明

## 功能概述

已在现有底盘控制代码基础上添加了**机械臂视觉伺服控制**功能:

- **三个机械臂电机**(地址 5/6/7)与底盘电机(地址 1-4)共用 Serial2 总线
- **视觉伺服模式**: 按住 A 键,根据外部视觉计算机发来的误差自动调整角度和前后电机
- **自动抓取流程**: 按住 A 键时,上下电机自动执行"下降→等待→上升"流程
- **底盘控制不受影响**: 左摇杆 XY 和右摇杆 X 仍控制底盘移动

---

## 硬件连接

### 电机总线(Serial2)
- **RX**: GPIO 16
- **TX**: GPIO 17
- **波特率**: 115200
- **电机地址分配**:
  - 1-4: 底盘四个轮子(已有)
  - **5: 角度电机**(机械臂旋转)
  - **6: 上下电机**(机械臂升降)
  - **7: 前后电机**(机械臂伸缩)

### 视觉数据串口(Serial1)
- **RX**: GPIO 18
- **TX**: GPIO 19
- **波特率**: 115200
- **协议格式**: `"A:<角度误差>,F:<前后误差>\n"`
  - 示例: `"A:12.5,F:-8.3\n"`
  - 角度误差正值 = 需要顺时针转
  - 前后误差正值 = 需要向前伸

---

## 手柄按键映射

### 底盘控制(不变)
- **左摇杆 Y**: 前后速度
- **左摇杆 X**: 左右平移速度
- **右摇杆 X**: 底盘旋转角速度

### 机械臂控制 (更新于 2026/05/31)
- **A 键(按住)**: 触发自动抓取流程
  - **视觉伺服**: 角度和前后电机持续根据视觉误差调整(对准目标)
  - **自动上下**: 上下电机自动执行完整流程
    1. 下降到 5 圈位置
    2. 等待 5 秒(模拟夹紧)
    3. 回升到 1 圈位置
  - 松开 A 键立即停止所有动作并重置流程

---

## 控制逻辑

### 自动抓取流程(按住 A 键时)

**时间轴**:
```
0s    按下 A 键
      ├─ 视觉伺服开始(角度+前后电机持续调整)
      └─ 上下电机开始下降(-50 rpm)

~6s   到达底部(5圈位置)
      ├─ 上下电机停止
      └─ 开始等待(模拟夹紧动作)

~11s  等待结束
      └─ 上下电机开始上升(+50 rpm)

~16s  到达顶部(1圈位置)
      └─ 上下电机停止,流程完成

松开A 所有电机立即停止,流程重置
```

### 视觉伺服(与上下流程同步运行)
- **P 控制器**: `电机转速 = Kp × 误差`
- **死区**: 误差小于 5.0 时电机停止
- **速度限制**: 最大 100 rpm
- **默认增益**:
  - 角度轴 Kp = 30.0
  - 前后轴 Kp = 30.0

### 上下电机参数
- **下降目标**: 5 圈
- **上升目标**: 1 圈
- **移动速度**: 50 rpm
- **等待时间**: 5 秒

### 安全保护
- 手柄断开 → 底盘和机械臂全部停止
- 手柄超时(250ms 无新数据) → 底盘和机械臂全部停止
- 松开 A 键 → 所有机械臂电机立即停止,流程重置

---

## 代码结构

### 新增库文件
1. **`lib/ArmControl/ArmControl.h`**
   - 机械臂控制类
   - 管理三个电机(地址 5/6/7)
   - 提供视觉伺服接口

2. **`lib/VisionSerial/VisionSerial.h`**
   - 视觉串口通信类
   - 通过 Serial1 接收误差数据
   - 解析协议并提供非阻塞读取接口

### 修改的文件
1. **`lib/ControlSerial/ControlSerial.h`**
   - 新增 `set_position()` 方法(预留接口,待实现)
   - 用于未来的位置控制功能

2. **`include/InputState.h`**
   - 新增 `armServoTrigger` 字段(A 键状态)

3. **`lib/GamepadInput/GamepadInput.cpp`**
   - 在 `fillArmCommand()` 里映射 A 按键

4. **`src/main.cpp`**
   - 初始化机械臂和视觉串口
   - 在 `applyArmCommand()` 里实现自动抓取流程状态机

---

## 调试和调参

### 串口输出
- **USB 串口(Serial)**: 调试信息和手柄状态
- **波特率**: 115200
- 可以看到手柄连接状态、底盘速度命令、按键状态

**按 A 键时会显示**:
```
Starting descent to 5 rotations
Reached bottom, waiting 5 seconds
Starting ascent to 1 rotation
Reached top, sequence complete
```

### 调整自动流程参数
修改 `main.cpp` 里的常量:
```cpp
constexpr float kDescendTargetRotations = 5.0f;   // 下降目标圈数
constexpr float kAscendTargetRotations = 1.0f;    // 上升目标圈数
constexpr uint32_t kGripDelayMs = 5000;           // 底部等待时间(毫秒)
constexpr float kVerticalMoveSpeed = 50.0f;       // 上下移动速度(rpm)
```

### 调整视觉伺服参数
在 `ArmControl.h` 里修改:
```cpp
float kp_angle = 30.0f;       // 角度轴增益
float kp_forward = 30.0f;     // 前后轴增益
float deadzone = 5.0f;        // 死区阈值
float max_rpm = 100.0f;       // 最大转速
```

或在 `main.cpp` 的 `setup()` 里动态设置:
```cpp
g_arm_control->setAnglePID(30.0f, 0.5f, 5.0f);
g_arm_control->setForwardPID(30.0f, 0.5f, 5.0f);
g_arm_control->setDeadzone(5.0f);
g_arm_control->setMaxRpm(100.0f);
```

### 修改视觉串口引脚
修改 `main.cpp` 里的常量:
```cpp
constexpr int kVisionRxPin = 18;  // 改成实际 RX 引脚
constexpr int kVisionTxPin = 19;  // 改成实际 TX 引脚
```

---

## 重要说明

### ⚠️ 当前使用速度控制(临时方案)

由于位置控制接口(`set_position`)尚未实现,当前使用**基于时间估算**的方式:

- **下降**: 5圈 @ 50rpm ≈ 6秒
- **上升**: 4圈 @ 50rpm ≈ 5秒

**这不是精确的位置控制!** 实际位置会受以下因素影响:
- 电机负载
- 电压波动
- 启动/停止延迟

### 调整时间估算

如果实际运行时间不准确,修改 `main.cpp` 里的判断条件:

```cpp
case DESCENDING:
    // 当前估算: 6000ms
    if ((currentTime - g_last_fresh_input_ms) > 6000) {
        // 改成实际测量的时间,比如 7000ms

case ASCENDING:
    // 当前估算: 5000ms
    if ((currentTime - g_grip_start_time - kGripDelayMs) > 5000) {
        // 改成实际测量的时间
```

---

## 待实现功能

### 位置控制(预留接口)

一旦实现了 `ControlSerial::set_position()`,代码里标注了 `TODO` 的地方需要替换:

```cpp
// 当前(临时):
g_arm_control->manualVertical(-50.0f);  // 速度控制,靠时间估算

// 未来(精确):
g_arm_control->setVerticalPosition(5.0 * 360);  // 位置控制,精确到度数
```

实现步骤:
1. 在 `lib/ControlSerial/ControlSerial.h` 的 `set_position()` 方法里实现协议
2. 参考注释里的协议结构(命令 0xFD)
3. 替换 `main.cpp` 里所有 `TODO` 标记的地方

### 可能的改进方向
- PID 控制器(当前只有 P)
- 位置反馈(从电机读取当前位置)
- 视觉数据超时保护
- 多目标点切换
- 通过串口命令实时调参
- 记录和回放轨迹
- 错误处理(超时、失败重试)

---

## 常见问题

### Q: 按 A 键没反应?
1. 检查手柄是否连接(串口显示 "Controller connected")
2. 检查串口是否显示 "Starting descent"
3. 检查电机地址是否正确(地址 5/6/7)

### Q: 上下电机一直转不停?
**原因**: 时间估算不准确,没有触发停止条件

**临时解决**: 松开 A 键立即停止

**永久解决**: 
1. 测量实际运行时间
2. 修改代码里的时间判断
3. 或实现位置控制接口

### Q: 视觉伺服不工作?
1. 检查 Serial1 连线(GPIO 18/19)
2. 检查视觉计算机是否在发送数据
3. 检查协议格式是否为 `"A:<angle>,F:<forward>\n"`
4. 打开串口监视器看是否有报错

### Q: 电机地址冲突?
- 底盘占用 1-4,机械臂占用 5-7
- 用电机调试软件确认每个电机的地址设置
- 参考 `MotorSettings.txt` 设置电机参数

### Q: 底盘控制受影响?
- 机械臂和底盘共用 Serial2,但地址不同,理论上不会冲突
- 如果出现问题,检查电机地址是否重复

---

## 测试步骤

### 1. 不接电机测试(串口监视器)

上传代码后,打开串口监视器(波特率 115200):

```
1. 按住 A 键
   → 应该看到: "Starting descent to 5 rotations"

2. 等待约6秒
   → 应该看到: "Reached bottom, waiting 5 seconds"

3. 再等待5秒
   → 应该看到: "Starting ascent to 1 rotation"

4. 再等待约5秒
   → 应该看到: "Reached top, sequence complete"

5. 松开 A 键
   → 应该看到: "A button released, stopping sequence"
```

### 2. 接电机测试

**安全提示**: 
- 确保机械臂有足够的运动空间
- 准备好随时松开 A 键紧急停止
- 第一次测试时降低速度(改 `kVerticalMoveSpeed` 为 20)

**观察**:
- 按住 A 键,上下电机应该开始向下转
- 同时角度和前后电机根据视觉误差调整(如果有视觉数据)
- 约6秒后上下电机停止
- 等待5秒
- 上下电机开始向上转
- 约5秒后停止

### 3. 测试视觉数据接收(可选)

如果你有另一个设备(Arduino/树莓派/电脑串口工具):

**连接**:
- 设备 TX → ESP32 GPIO 18 (RX)
- 设备 GND → ESP32 GND

**发送测试数据**(波特率115200):
```
A:10.5,F:-5.2
A:8.3,F:-3.1
A:2.1,F:0.5
```

**按住 A 键时**,角度和前后电机会根据误差转动(如果接了电机)。

---

## 编译和上传

```bash
# 编译
pio run

# 上传到 ESP32
pio run --target upload

# 打开串口监视器
pio device monitor
```

或在 VS Code 里使用 PlatformIO 扩展的按钮。

---

## 技术支持

如有问题或需要修改功能,参考:
- `ARM_CONTROL_PLAN.md`: 详细实现计划
- `INTEGRATION_NOTES.md`: 原有底盘集成说明
- 代码注释: 每个类和方法都有详细说明
