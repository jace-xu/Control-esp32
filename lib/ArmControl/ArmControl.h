#pragma once
#include <ControlSerial.h>
#include <Arduino.h>
#include <cmath>

/**
 * @brief Arm control class for visual servoing and manual control
 * @note Manages 3 motors via ControlSerial: angle (addr 5), vertical (addr 6), forward (addr 7)
 * @note Addresses 5-7 share the same Serial2 bus with chassis motors (addresses 1-4)
 * @note 夹爪由 ESP32 LEDC PWM 驱动一个舵机 (grip/release 到固定角度)
 * @note [ ! ] Only can be created once in the whole program
 * @note == Version 2.0.0 ==
 */
class ArmControl {
private:
    // 三个机械臂电机的总线地址 (与底盘电机 1~4 错开)
    static constexpr int kAngleAddr = 5;      // 角度电机: 旋转对准
    static constexpr int kVerticalAddr = 6;   // 上下电机: 升降
    static constexpr int kForwardAddr = 7;    // 前后电机: 进给对准

    // ---- 夹爪舵机 (ESP32 LEDC PWM) ----
    // [ ! ] 以下引脚/角度为占位默认值, 请按实际硬件修改
    static constexpr int kGripperPin = 13;        // 舵机信号引脚 (GPIO)
    static constexpr int kGripperPwmChannel = 4;  // LEDC 通道 (避开底盘可能占用的 0~3)
    static constexpr int kGripperPwmFreq = 50;    // 舵机标准 50Hz
    static constexpr int kGripperPwmResBits = 16; // PWM 分辨率位数
    static constexpr float kGripperClosedAngle = 100.0f;   // 夹紧角度 (度)
    static constexpr float kGripperOpenAngle = 180.0f;    // 松开角度 (度)
    // 舵机脉宽范围 (微秒): 多数舵机 0.5ms~2.5ms 对应 0~180 度
    static constexpr float kGripperMinPulseUs = 500.0f;
    static constexpr float kGripperMaxPulseUs = 2500.0f;

    ControlSerial* control_serial = nullptr;  // 共用的 Serial2 命令通道 (单例)


    // PID controller parameters (tunable)
    float kp_angle = 30.0f;       // P gain for angle servo
    float ki_angle = 0.0f;        // I gain for angle servo (积分项)
    float kd_angle = 0.0f;        // D gain for angle servo (微分项)

    float kp_forward = 30.0f;     // P gain for forward servo
    float ki_forward = 0.0f;      // I gain for forward servo
    float kd_forward = 0.0f;      // D gain for forward servo

    float deadzone = 5.0f;        // Error deadzone threshold
    float max_rpm = 100.0f;       // Maximum motor speed limit

    // PID state variables (internal)
    float angle_error_integral = 0.0f;     // 角度误差积分
    float angle_error_last = 0.0f;         // 上一次角度误差(用于微分)
    float forward_error_integral = 0.0f;   // 前后误差积分
    float forward_error_last = 0.0f;       // 上一次前后误差

    uint32_t last_update_time = 0;         // 上次更新时间(用于计算dt)

    // Anti-windup limits (防止积分饱和)
    float integral_limit = 1000.0f;        // 积分项限制

    // 电机位置移动速度 (rpm), setPosition 时使用 (角度/上下电机共用)
    static constexpr float kPosSpeed = 50.0f;

    // 位置问询后等待电机回复的时间 (ms), readVerticalPosition 时使用
    // 给电机收到问询、处理、回 8 字节的往返留出时间, 否则 read 会提前超时
    static constexpr uint32_t kPositionReadDelayMs = 2;

public:
    // Disabled copying and assignment
    ArmControl(const ArmControl&) = delete;
    ArmControl& operator=(const ArmControl&) = delete;

    /**
     * @brief Create arm control object
     * @note Motors share the single Serial2 bus via ControlSerial::get_instance()
     * @note 同时初始化夹爪舵机的 LEDC PWM, 上电默认松开
     */
    ArmControl() {
        this->control_serial = &(ControlSerial::get_instance());
        // 初始化夹爪舵机 PWM (ESP32 Arduino core 2.x: 通道制 LEDC)
        ledcSetup(kGripperPwmChannel, kGripperPwmFreq, kGripperPwmResBits);
        ledcAttachPin(kGripperPin, kGripperPwmChannel);
        release();  // 上电默认松开夹爪
    }

    /// @brief Destruct the object
    ~ArmControl() {
        ledcDetachPin(kGripperPin);
    }

public:
    // ========================================================================
    // 夹爪舵机控制
    // ========================================================================

    /// @brief 夹紧夹爪 (舵机转到夹紧角度)
    void grip() {
        setGripperAngle(kGripperClosedAngle);
    }

    /// @brief 松开夹爪 (舵机转到松开角度)
    void release() {
        setGripperAngle(kGripperOpenAngle);
    }

public:
    /// @brief Stop all arm motors
    /// @note Also resets PID state so a later servo session starts clean.
    void stop() {
        this->control_serial->thread_lock();
        this->control_serial->generate_stop_command(kAngleAddr);
        this->control_serial->send_command();
        this->control_serial->generate_stop_command(kVerticalAddr);
        this->control_serial->send_command();
        this->control_serial->generate_stop_command(kForwardAddr);
        this->control_serial->send_command();
        this->control_serial->thread_unlock();
        resetPID();
    }

    /// @brief Stop only servo motors (angle and forward), keep vertical motor state
    /// @note Also resets PID state (integral/derivative) to prevent windup on resume.
    ///       Vertical motor is speed-controlled and not affected by resetPID().
    void stopServo() {
        this->control_serial->thread_lock();
        this->control_serial->generate_stop_command(kAngleAddr);
        this->control_serial->send_command();
        this->control_serial->generate_stop_command(kForwardAddr);
        this->control_serial->send_command();
        this->control_serial->thread_unlock();
        resetPID();
    }

    /**
     * @brief Update visual servoing control (PID controller)
     * @param angleError Error in angle axis (positive = need to rotate CW)
     * @param forwardError Error in forward axis (positive = need to move forward)
     * @note Uses PID control: rpm = Kp*e + Ki*∫e*dt + Kd*de/dt
     * @note Stops motor if error is within deadzone
     */
    void updateVisualServo(float angleError, float forwardError) {
        // 计算时间间隔 dt (秒)
        uint32_t current_time = millis();
        float dt = 0.02f;  // 默认 20ms (50Hz)
        if (last_update_time > 0) {
            dt = (current_time - last_update_time) / 1000.0f;
            if (dt > 0.1f) dt = 0.1f;      // 上界:防止长间隔导致积分/微分异常
            if (dt < 0.001f) dt = 0.001f;  // 下界:防止同毫秒重复调用时 dt=0 导致 D 项除零(inf/NaN)
        }
        last_update_time = current_time;

        // ========== 角度轴 PID 控制 ==========
        float angle_rpm = 0.0f;

        if (fabsf(angleError) > deadzone) {
            // P 项: 比例控制
            float p_term = kp_angle * angleError;

            // I 项: 积分控制(累积误差)
            angle_error_integral += angleError * dt;
            // 积分限幅,防止积分饱和
            if (angle_error_integral > integral_limit) {
                angle_error_integral = integral_limit;
            } 
            else if (angle_error_integral < -integral_limit) {
                angle_error_integral = -integral_limit;
            }
            float i_term = ki_angle * angle_error_integral;

            // D 项: 微分控制(误差变化率)
            float error_derivative = (angleError - angle_error_last) / dt;
            float d_term = kd_angle * error_derivative;

            // PID 输出
            angle_rpm = p_term + i_term + d_term;

            // 输出限幅
            angle_rpm = fmaxf(-max_rpm, fminf(max_rpm, angle_rpm));

            // 更新上一次误差
            angle_error_last = angleError;
        } else {
            // 进入死区,清零积分项,防止积分累积
            angle_error_integral = 0.0f;
            angle_error_last = 0.0f;
        }

      
        // ========== 前后轴 PID 控制 ==========
        float forward_rpm = 0.0f;

        if (fabsf(forwardError) > deadzone) {
            // P 项
            float p_term = kp_forward * forwardError;

            // I 项
            forward_error_integral += forwardError * dt;
            if (forward_error_integral > integral_limit) {
                forward_error_integral = integral_limit;
            } else if (forward_error_integral < -integral_limit) {
                forward_error_integral = -integral_limit;
            }
            float i_term = ki_forward * forward_error_integral;

            // D 项
            float error_derivative = (forwardError - forward_error_last) / dt;
            float d_term = kd_forward * error_derivative;

            // PID 输出
            forward_rpm = p_term + i_term + d_term;

            // 输出限幅
            forward_rpm = fmaxf(-max_rpm, fminf(max_rpm, forward_rpm));

            // 更新上一次误差
            forward_error_last = forwardError;
        } else {
            // 进入死区,清零积分项
            forward_error_integral = 0.0f;
            forward_error_last = 0.0f;
        }

        // 角度 + 前后 两条速度命令合并为一条长命令一次性下发:
        // 单次总线事务, 两个伺服电机更同步, 也省去逐条 flush 的开销。
        // 上下电机已改用位置控制(设一次), 不在此每帧重发。
        // 全程持锁, 防止 append 序列被其他线程的命令插入而错帧。
        this->control_serial->thread_lock();
        this->control_serial->clear_long_command();
        this->control_serial->X_generate_set_rotate_speed_command(kAngleAddr, angle_rpm);
        this->control_serial->append_command();
        this->control_serial->X_generate_set_rotate_speed_command(kForwardAddr, forward_rpm);
        this->control_serial->append_command();
        this->control_serial->send_long_command();
        this->control_serial->thread_unlock();
    }

    /**
     * @brief Manual control for vertical motor (speed mode)
     * @param rpm Rotation speed, positive = up, negative = down
     */
    void manualVertical(float rpm) {
        this->control_serial->thread_lock();
        this->control_serial->X_generate_set_rotate_speed_command(kVerticalAddr, rpm);
        this->control_serial->send_command();
        this->control_serial->thread_unlock();
    }

    /**
     * @brief 通用电机位置控制 (绝对角度, 原样下发不取负)
     * @param address        目标电机总线地址 (kAngleAddr / kVerticalAddr)
     * @param positionDegrees 目标绝对角度 (度), 相对上电零点
     * @note  使用 X_generate_set_position_command (0xFD), 电机自走到目标位置
     * @note  方向约定交给调用方: 正角度→电机一个方向, 负角度→反方向 (底层命令天然行为)
     * @note  内部不取负, 设一次即可, 电机内部闭环完成移动
     */
    void setPosition(int address, float positionDegrees) {
        this->control_serial->thread_lock();
        this->control_serial->X_generate_set_position_command(
            address, positionDegrees, kPosSpeed);
        this->control_serial->send_command();
        this->control_serial->thread_unlock();
    }

    /// @brief 上下电机位置控制 (绝对角度), setPosition 的薄封装
    void setVerticalPosition(float positionDegrees) {
        setPosition(kVerticalAddr, positionDegrees);
    }

    /// @brief 角度电机位置控制 (绝对角度), setPosition 的薄封装
    /// @note  供 PRE_CATCH 预备摆位使用 (角度电机平时是 PID 速度控制)
    void setAnglePosition(float positionDegrees) {
        setPosition(kAngleAddr, positionDegrees);
    }

    /// @brief 前后电机位置控制 (绝对角度), setPosition 的薄封装
    /// @note  供阶段1 收臂归位使用 (前后电机平时是 PID 速度控制)
    void setForwardPosition(float positionDegrees) {
        setPosition(kForwardAddr, positionDegrees);
    }

    /**
     * @brief 通用电机当前绝对位置读取 (位置反馈)
     * @param address      目标电机总线地址 (kAngleAddr / kVerticalAddr / kForwardAddr)
     * @param currentDegrees [out] 当前角度 (度), 相对上电零点; 失败时不变
     * @return true=读取成功, false=超时/校验失败
     * @note  先发问询命令, 延迟 kPositionReadDelayMs 等电机回复再读响应; 全程持锁为原子事务,
     *        防延迟期间其他线程命令插入/把响应读走。
     * @note  [ ! ] 被读电机的驱动器"响应"须开启 (0x36 问询/响应协议), 否则读不到。
     */
    bool readPosition(int address, float& currentDegrees) {
        this->control_serial->thread_lock();
        this->control_serial->ask_current_position(address);
        delay(kPositionReadDelayMs);  // 等电机把响应放进硬件接收缓冲
        bool ok = this->control_serial->X_read_current_position(currentDegrees);
        this->control_serial->thread_unlock();
        return ok;
    }

    /// @brief 读上下电机当前位置 (readPosition 薄封装)
    bool readVerticalPosition(float& currentDegrees) {
        return readPosition(kVerticalAddr, currentDegrees);
    }

    /// @brief 读角度电机当前位置 (readPosition 薄封装)
    bool readAnglePosition(float& currentDegrees) {
        return readPosition(kAngleAddr, currentDegrees);
    }

    /// @brief 读前后电机当前位置 (readPosition 薄封装)
    bool readForwardPosition(float& currentDegrees) {
        return readPosition(kForwardAddr, currentDegrees);
    }

    /**
     * @brief Set PID gains for angle axis
     * @param kp Proportional gain (rpm per error unit)
     * @param ki Integral gain (rpm per accumulated error)
     * @param kd Derivative gain (rpm per error rate)
     */
    void setAnglePID(float kp, float ki, float kd) {
        this->kp_angle = kp;
        this->ki_angle = ki;
        this->kd_angle = kd;
    }

    /**
     * @brief Set PID gains for forward axis
     * @param kp Proportional gain
     * @param ki Integral gain
     * @param kd Derivative gain
     */
    void setForwardPID(float kp, float ki, float kd) {
        this->kp_forward = kp;
        this->ki_forward = ki;
        this->kd_forward = kd;
    }


    /**
     * @brief Set error deadzone threshold
     * @param dz Deadzone value (motor stops if |error| < dz)
     */
    void setDeadzone(float dz) {
        this->deadzone = dz;
    }

    /**
     * @brief Set maximum motor speed limit
     * @param max Maximum RPM
     */
    void setMaxRpm(float max) {
        this->max_rpm = max;
    }

    /**
     * @brief Set integral limit (anti-windup)
     * @param limit Maximum absolute value for integral term
     */
    void setIntegralLimit(float limit) {
        this->integral_limit = limit;
    }

    /**
     * @brief Reset PID state (clear integral and derivative terms)
     * @note Call this when starting a new servo session
     */
    void resetPID() {
        angle_error_integral = 0.0f;
        angle_error_last = 0.0f;
        forward_error_integral = 0.0f;
        forward_error_last = 0.0f;
        last_update_time = 0;
    }

private:
    /**
     * @brief 驱动夹爪舵机到指定角度
     * @param angle 目标角度 (度), 会被限幅到 [0, 180]
     * @note  角度 → 脉宽(us) → LEDC 占空比, 50Hz 周期为 20000us
     */
    void setGripperAngle(float angle) {
        if (angle < 0.0f) angle = 0.0f;
        if (angle > 180.0f) angle = 180.0f;
        // 角度线性映射到脉宽
        float pulse_us = kGripperMinPulseUs +
            (kGripperMaxPulseUs - kGripperMinPulseUs) * (angle / 180.0f);
        // 脉宽 → 占空比: duty = pulse / period * (2^bits - 1), 周期 = 1e6/freq us
        const float period_us = 1000000.0f / kGripperPwmFreq;
        const uint32_t max_duty = (1u << kGripperPwmResBits) - 1u;
        uint32_t duty = static_cast<uint32_t>((pulse_us / period_us) * max_duty);
        ledcWrite(kGripperPwmChannel, duty);
    }
};
