# 机械臂视觉伺服控制实现计划

## 系统架构判断

这是**视觉伺服(visual servoing)**系统,不是手柄直控关节:
- 外部视觉计算机通过 **Serial1(新 UART)** 发送"前后误差 + 角度误差"
- 按住手柄某键进入伺服模式,ESP32 根据误差驱动两个电机(P 闭环)
- 上下电机独立手动控制,走到指定位置(需位置控制命令)
- 三个机械臂电机和底盘**共用 Serial2 电机总线**,只是地址不同(底盘 1-4,机械臂 5/6/7)

## 实现方案(五个模块)

### 1. 扩展 ControlSerial 库(预留位置控制接口)

**文件**: `lib/ControlSerial/ControlSerial.h`

**新增方法**:
```cpp
/**
 * @brief Set motor to absolute position (RESERVED - implementation pending)
 * @param address Motor address
 * @param position Target position in degrees or encoder counts
 * @note Protocol command 0xFD, implementation to be filled by user
 */
void set_position(int address, float position);
```

**实现**: 在 `.h` 文件里添加方法声明和空实现(inline),注释标注"RESERVED - user to implement"。不改现有 `set_rpm`/`stop` 逻辑。

---

### 2. 创建 ArmControl 库

**文件**: `lib/ArmControl/ArmControl.h`

**职责**:
- 管理三个机械臂电机(地址 5=角度, 6=上下, 7=前后)
- 提供视觉伺服接口:`updateVisualServo(float angleError, float forwardError)`
- 提供手动控制接口:`manualVertical(float speed)` / `stop()`
- P 控制器参数可调

**关键设计**:
```cpp
class ArmControl {
private:
    ControlSerial* control_serial;
    Motor* angle_motor;      // 地址 5
    Motor* vertical_motor;   // 地址 6
    Motor* forward_motor;    // 地址 7
    
    float kp_angle = 50.0f;     // P gain for angle (rpm per error unit)
    float kp_forward = 50.0f;   // P gain for forward
    float deadzone = 5.0f;      // Error deadzone
    float max_rpm = 200.0f;     // Speed limit

public:
    ArmControl(ControlSerial& serial);
    ~ArmControl();
    
    void updateVisualServo(float angleError, float forwardError);
    void manualVertical(float speed);  // For manual up/down control
    void setVerticalPosition(float position);  // Calls set_position (reserved)
    void stop();
};
```

**P 控制逻辑**:
```
rpm = clamp(kp * error, -max_rpm, max_rpm)
if (abs(error) < deadzone) rpm = 0
```

---

### 3. 创建 VisionSerial 库

**文件**: `lib/VisionSerial/VisionSerial.h`

**职责**:
- 通过 Serial1 接收视觉计算机发来的误差数据
- 解析协议,提供 `read()` 接口返回误差值

**协议假设**(需用户确认):
```
格式: "A:<angle_error>,F:<forward_error>\n"
示例: "A:12.5,F:-8.3\n"
```

**接口**:
```cpp
struct VisionError {
    bool valid = false;
    float angleError = 0.0f;
    float forwardError = 0.0f;
    uint32_t timestampMs = 0;
};

class VisionSerial {
public:
    VisionSerial(int baudRate, int rxPin, int txPin);
    VisionError read();  // Non-blocking, returns latest parsed data
};
```

**引脚假设**(需用户确认):
- RX: GPIO 18
- TX: GPIO 19
- 波特率: 115200

---

### 4. 扩展 InputState 和 GamepadInput

**文件**: `include/InputState.h`

**新增字段**:
```cpp
struct InputState {
    // ... existing fields ...
    bool armServoTrigger = false;  // True when servo button held
    float armVerticalManual = 0.0f;  // -1.0 ~ +1.0 for manual up/down
};
```

**文件**: `lib/GamepadInput/GamepadInput.cpp`

**按键映射**(需用户确认具体按键):
- **伺服触发**: A 键按住 → `armServoTrigger = true`
- **上下手动**: L1(上) / R1(下) → `armVerticalManual = ±1.0`

在 `fillArmCommand` 里读取这些按键状态并填充。

---

### 5. 修改 main.cpp(核心逻辑)

**新增全局对象**:
```cpp
VisionSerial* g_vision_serial = nullptr;
ArmControl* g_arm_control = nullptr;
```

**setup() 初始化**:
```cpp
g_vision_serial = new VisionSerial(115200, 18, 19);
g_arm_control = new ArmControl(*g_control_serial);  // 复用底盘的 Serial2
```

**applyArmCommand() 实现**:
```cpp
void applyArmCommand(const InputState& input) {
    // 1. 手动上下控制(优先级高,随时可用)
    if (fabsf(input.armVerticalManual) > 0.1f) {
        g_arm_control->manualVertical(input.armVerticalManual * 100.0f);  // 映射到 rpm
    }
    
    // 2. 视觉伺服模式(按住触发键时)
    if (input.armServoTrigger) {
        VisionError visionError = g_vision_serial->read();
        if (visionError.valid) {
            g_arm_control->updateVisualServo(visionError.angleError, visionError.forwardError);
        } else {
            // 无有效视觉数据时停止伺服轴
            g_arm_control->stop();  // 只停角度和前后,不影响手动上下
        }
    } else {
        // 松开触发键,停止伺服轴
        g_arm_control->stop();
    }
}
```

---

## 关键假设(需用户确认)

### ⚠️ 必须确认的参数

1. **Serial1 引脚**:
   - RX: GPIO 18 ?
   - TX: GPIO 19 ?
   - 波特率: 115200 ?

2. **视觉数据协议**:
   - 当前假设: `"A:<angle>,F:<forward>\n"` 文本格式
   - 实际格式是什么?(二进制?JSON?其他?)
   - 误差单位是什么?(像素?归一化?角度?)

3. **手柄按键映射**:
   - 伺服触发: A 键 ?
   - 上下手动: L1(上) / R1(下) ?

4. **电机地址分配**:
   - 地址 5: 角度电机
   - 地址 6: 上下电机
   - 地址 7: 前后电机
   - 是否正确?

5. **P 控制参数**(可后期调):
   - Kp_angle = 50.0
   - Kp_forward = 50.0
   - 死区 = 5.0
   - 最大转速 = 200 rpm

---

## 实现顺序

1. 扩展 `ControlSerial::set_position`(预留空实现)
2. 创建 `ArmControl` 库
3. 创建 `VisionSerial` 库
4. 修改 `InputState.h` 和 `GamepadInput.cpp`
5. 修改 `main.cpp`
6. 编译验证

---

## 验证计划

1. **编译通过**: PlatformIO build
2. **底盘功能不受影响**: 左右摇杆仍能控制底盘
3. **手动上下**: 按 L1/R1 时上下电机转动
4. **视觉伺服**: 按住 A 键,Serial1 收到数据时角度和前后电机响应
5. **安全停止**: 松开 A 键或手柄断开时机械臂立即停止

---

## 后续扩展点

- 位置控制协议实现(用户填充 `set_position`)
- PID 控制器(当前只有 P)
- 多目标点切换
- 视觉数据超时保护
- 调参界面(通过 Serial 命令)