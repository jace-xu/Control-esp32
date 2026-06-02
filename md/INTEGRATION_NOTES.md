# 手柄-底盘集成说明

## 1. 这次改了什么

当前工程已经从“手柄测试代码”整理成“单块 ESP32 直控底盘”的最小结构。

主要文件如下：

- `src/main.cpp`
  系统主流程。只负责初始化、循环调度、停车保护、调用底盘/机械臂接口。

- `src/GamepadInput.cpp`
  手柄输入解析层。负责从 Bluepad32 读取摇杆、扳机、按钮，并整理成统一输入结构。

- `include/GamepadInput.h`
  手柄输入层对外接口。

- `include/InputState.h`
  统一输入数据结构定义。后续底盘、机械臂都从这里取数据。

- `BottomControl.h / Motor.h / ControlSerial.h`
  底盘控制库，本次没有改它的控制逻辑。

## 2. 现在已经具备的功能

- 通过 Bluepad32 连接蓝牙手柄
- 读取摇杆、扳机、主要按钮
- 左摇杆 Y 控制前后速度 `vx`
- 左摇杆 X 控制左右平移速度 `vy`
- 右摇杆 X 控制底盘角速度 `wz`
- 对摇杆做死区处理
- 将摇杆量线性映射为底盘控制量
- 手柄断开自动停车
- 手柄超时无新数据自动停车

## 3. 当前输入结构怎么用

所有手柄输入已经统一封装在 `InputState` 里。

常用字段：

- `input.connected`
  当前是否有可用手柄

- `input.hasFreshData`
  当前循环是否收到了新一帧手柄数据

- `input.timedOut`
  手柄虽然还连着，但是否已经超时没新数据

- `input.sticks.lx / ly / rx / ry`
  左右摇杆原始值

- `input.triggers.left / right`
  左右扳机原始值

- `input.buttons.a / b / x / y`
  面键

- `input.buttons.l1 / r1`
  肩键

- `input.buttons.start / select`
  开始键、选择键

- `input.buttons.thumbL / thumbR`
  摇杆按下

- `input.buttons.dpadX / dpadY`
  十字键方向

- `input.chassis.vx_mm_s`
  底盘前后速度，单位 `mm/s`

- `input.chassis.vy_mm_s`
  底盘左右速度，单位 `mm/s`

- `input.chassis.wz_rad_s`
  底盘角速度，单位 `rad/s`

## 4. main 里后续怎么接机械臂

当前 `main.cpp` 里已经预留：

```cpp
void applyArmCommand(const InputState& input) {
    (void)input;
}
```

后续如果机械臂也用同一个手柄控制，建议这样扩展：

1. 在 `GamepadInput.cpp` 里约定机械臂控制规则
   例如：
   - `LB/RB` 控制某关节正反转
   - `LT/RT` 控制夹爪开合
   - `十字键` 控制模式切换

2. 把机械臂命令写入 `input.arm`

3. 在 `applyArmCommand(const InputState& input)` 里读取 `input.arm`
   再调用机械臂电控对象的方法

这样底盘和机械臂共用同一份输入状态，职责比较清楚。

## 5. 后续最常用的入口

如果只是想在现有工程上继续写功能，优先看：

- `src/main.cpp`
  看系统主流程和调用点

- `include/InputState.h`
  整理好的手柄输入字段

- `src/GamepadInput.cpp`
  按钮/摇杆映射方式


## 6. 当前串口/底盘相关约定

- 底盘控制走 `Serial2`
- 当前代码定义：
  - `GPIO16` = `RX`
  - `GPIO17` = `TX`
- 底盘控制库里默认电机地址使用 `1 ~ 4`
- 电机参数需要提前按 `MotorSettings.txt` 设置好

## 7. 当前 Bluepad32 版本注意事项

当前环境里的 Bluepad32 不是所有资料里的同一套 API。

本工程已经按本机实际版本兼容处理：

- `start` 使用 `miscStart()`
- `select` 使用 `miscSelect()`
- 十字键方向使用 `dpad()` 位掩码解析

所以后续如果继续改手柄按钮逻辑，可能需要参考本地头文件：
`C:\Users\10200\.platformio\packages\framework-arduinoespressif32\tools\sdk\esp32\include\bluepad32_arduino\include\ArduinoController.h`

