# 视觉串口二进制协议测试指南

## 协议格式

```
字节0: 0xAA (帧头)
字节1: ID (uint8, 0-255)
字节2-3: 数据1 (int16 小端) - 角度误差,范围 ±3000
字节4-5: 数据2 (int16 小端) - 前后误差,范围 ±3000

总共: 6 字节
```

### 小端模式说明

小端(Little-Endian): 低字节在前,高字节在后

```
示例: 数值 10 (0x000A)
小端: 0A 00  (低字节 0A 在前)

示例: 数值 -10 (0xFFF6, 补码)
小端: F6 FF  (低字节 F6 在前)

示例: 数值 1000 (0x03E8)
小端: E8 03
```

---

## 测试数据包示例

### 示例1: 角度误差=10, 前后误差=-10

```
十六进制: AA 01 0A 00 F6 FF

解析:
- 0xAA: 帧头
- 0x01: ID = 1
- 0x0A 0x00: 数据1 = 10 (小端)
- 0xF6 0xFF: 数据2 = -10 (小端,补码)
```

### 示例2: 角度误差=100, 前后误差=200

```
十六进制: AA 02 64 00 C8 00

解析:
- 0xAA: 帧头
- 0x02: ID = 2
- 0x64 0x00: 数据1 = 100
- 0xC8 0x00: 数据2 = 200
```

### 示例3: 角度误差=-500, 前后误差=1000

```
十六进制: AA 03 0C FE E8 03

解析:
- 0xAA: 帧头
- 0x03: ID = 3
- 0x0C 0xFE: 数据1 = -500 (0xFE0C 补码)
- 0xE8 0x03: 数据2 = 1000 (0x03E8)
```

---

## 测试方法

### 方法1: 用 Python 脚本发送测试数据

**文件**: `test_vision_serial.py`

```python
import serial
import struct
import time

# 配置串口(连接到 ESP32 的 GPIO18)
ser = serial.Serial('COM7', 115200, timeout=1)  # 改成你的端口号
time.sleep(2)

def send_vision_packet(id, angle_error, forward_error):
    """
    发送视觉误差数据包
    id: 0-255
    angle_error: -3000 ~ 3000
    forward_error: -3000 ~ 3000
    """
    # 构造数据包: 帧头 + ID + 数据1(小端) + 数据2(小端)
    packet = struct.pack('<BbHH', 0xAA, id, angle_error & 0xFFFF, forward_error & 0xFFFF)
    ser.write(packet)
    print(f"Sent: ID={id}, Angle={angle_error}, Forward={forward_error}")

# 测试序列
print("开始测试...")

# 测试1: 小误差
send_vision_packet(1, 10, -10)
time.sleep(1)

# 测试2: 中等误差
send_vision_packet(2, 100, 200)
time.sleep(1)

# 测试3: 大误差
send_vision_packet(3, -500, 1000)
time.sleep(1)

# 测试4: 连续发送(模拟实时视觉)
for i in range(10):
    angle = 100 - i * 10  # 误差逐渐减小
    forward = 50 - i * 5
    send_vision_packet(i, angle, forward)
    time.sleep(0.1)

print("测试完成")
ser.close()
```

**运行**:
```bash
python test_vision_serial.py
```

---

### 方法2: 用串口助手发送十六进制

**工具**: SSCOM、串口调试助手、CoolTerm 等

**步骤**:
1. 打开串口助手
2. 选择端口(连接到 ESP32 GPIO18 的设备)
3. 波特率: 115200
4. 切换到"十六进制发送"模式
5. 输入测试数据,点击发送

**测试数据**:
```
AA 01 0A 00 F6 FF    (角度=10, 前后=-10)
AA 02 64 00 C8 00    (角度=100, 前后=200)
AA 03 0C FE E8 03    (角度=-500, 前后=1000)
```

---

### 方法3: 树莓派发送(实际使用)

**树莓派 Python 代码**:

```python
import serial
import struct
import cv2
import numpy as np

# 打开串口(连接到 ESP32)
ser = serial.Serial('/dev/ttyUSB0', 115200)

# 打开摄像头
cap = cv2.VideoCapture(0)

packet_id = 0

while True:
    ret, frame = cap.read()
    if not ret:
        break
    
    # 图像处理,计算目标位置
    # ... (你的视觉算法)
    
    # 假设计算出的误差
    target_x, target_y = 320, 240  # 屏幕中心
    object_x, object_y = detect_object(frame)  # 你的检测函数
    
    angle_error = int(object_x - target_x)
    forward_error = int(object_y - target_y)
    
    # 限制范围
    angle_error = max(-3000, min(3000, angle_error))
    forward_error = max(-3000, min(3000, forward_error))
    
    # 发送数据包
    packet = struct.pack('<BbHH', 0xAA, packet_id & 0xFF, 
                         angle_error & 0xFFFF, forward_error & 0xFFFF)
    ser.write(packet)
    
    packet_id += 1
    
    # 显示
    cv2.circle(frame, (object_x, object_y), 5, (0, 255, 0), -1)
    cv2.imshow('Vision', frame)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
ser.close()
cv2.destroyAllWindows()
```

---

## ESP32 串口监视器输出

### 正常情况

```
Chassis and arm control program started
Initialization complete
PID parameters: Kp=30.0, Ki=0.5, Kd=5.0
Controller connected, slot: 0

(按下 A 键)
Starting descent to 5 rotations
Vision: angle=10.00, forward=-10.00
Vision: angle=8.00, forward=-8.00
Vision: angle=5.00, forward=-5.00
Vision: angle=2.00, forward=-2.00
Vision: angle=0.00, forward=0.00

(6秒后)
Reached bottom, waiting 5 seconds
```

### 异常情况

**没有数据**:
```
Starting descent to 5 rotations
WARNING: No valid vision data
```

**数据超时**:
```
Vision: angle=10.00, forward=-10.00
(500ms 后没有新数据)
WARNING: No valid vision data
```

---

## 调试技巧

### 1. 查看原始字节

在 `VisionSerial::read()` 里添加调试输出:

```cpp
while (serial->available() > 0) {
    uint8_t byte = serial->read();
    
    // 调试: 打印接收到的字节
    Serial.printf("RX: 0x%02X\n", byte);
    
    // ... 原有代码 ...
}
```

### 2. 验证解析结果

在 `parsePacket()` 里添加:

```cpp
VisionError parsePacket(const uint8_t* packet) {
    // 调试: 打印完整数据包
    Serial.print("Packet: ");
    for (int i = 0; i < 6; i++) {
        Serial.printf("%02X ", packet[i]);
    }
    Serial.println();
    
    // ... 原有代码 ...
}
```

### 3. 检查数据范围

如果误差值异常,检查:
- 树莓派发送的是否是小端格式
- int16 补码是否正确
- 范围是否超过 ±3000

---

## 常见问题

### Q: 收不到数据?

**检查**:
1. 串口连线: 树莓派 TX → ESP32 GPIO18 (RX)
2. 共地: 树莓派 GND → ESP32 GND
3. 波特率: 两边都是 115200
4. 树莓派是否在发送数据

### Q: 数据解析错误?

**检查**:
1. 帧头是否是 0xAA
2. 字节序是否是小端
3. 负数是否用补码表示

### Q: 数据一直显示 valid=false?

**可能原因**:
1. 没有收到完整的 6 字节
2. 帧头不是 0xAA
3. 数据超出 ±3000 范围

---

## 性能指标

- **数据包大小**: 6 字节
- **波特率**: 115200 bps
- **理论最大帧率**: ~1920 fps (115200 / (6*10) )
- **实际推荐帧率**: 50-100 fps (足够视觉伺服)
- **延迟**: <10ms (串口传输 + 解析)

---

## 与树莓派对接清单

- [ ] 确认树莓派串口设备名(如 /dev/ttyUSB0)
- [ ] 确认波特率 115200
- [ ] 确认发送格式: 0xAA + ID + int16(LE) + int16(LE)
- [ ] 确认误差单位(像素?归一化?)
- [ ] 确认误差范围 ±3000
- [ ] 测试连续发送,观察 ESP32 串口输出
- [ ] 按 A 键,观察电机是否响应视觉误差

---

## 下一步

1. **上传代码到 ESP32**
2. **用 Python 脚本测试**: 发送几个测试数据包
3. **观察串口输出**: 确认数据正确解析
4. **连接树莓派**: 运行实际视觉程序
5. **按 A 键测试**: 观察机械臂是否根据视觉误差调整
6. **调整 PID 参数**: 根据实际效果优化
