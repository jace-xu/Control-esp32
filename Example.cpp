//#include <Arduino.h>
//#include <ControlSerial.h>
//#include <Motor.h>
#include <BottomControl.h>

ControlSerial control_serial(115200, 16, 17);
BottomControl bottom_control(control_serial);

void setup(){
    // Initialization
}

void loop(){
    // Main loop
    bottom_control.motors_control({100, 0, 0}); // 以100mm/s的速度向X轴正方向前进
    delay(5000);
    bottom_control.stop(); // 停止运动
    delay(5000);
}