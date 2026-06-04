#pragma once
#include <Motor.h>
#include <cmath>

/**
 * @brief Arm control class for visual servoing and manual control
 * @note Manages 3 motors: angle (address 5), vertical (address 6), forward (address 7)
 * @note Addresses 5-7 share the same Serial2 bus with chassis motors (addresses 1-4)
 * @note [ ! ] Only can be created once in the whole program
 * @note == Version 1.0.0 ==
 */
class ArmControl {
private:
    ControlSerial* control_serial = nullptr;
    Motor* angle_motor = nullptr;      // Address 5: rotation angle
    Motor* vertical_motor = nullptr;   // Address 6: up/down
    Motor* forward_motor = nullptr;    // Address 7: forward/backward

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

public:
    // Disabled copying and assignment
    ArmControl(const ArmControl&) = delete;
    ArmControl& operator=(const ArmControl&) = delete;

    /**
     * @brief Create arm control object
     * @note Motors share the single Serial2 bus via ControlSerial::get_instance()
     */
    ArmControl() {
        this->control_serial = &(ControlSerial::get_instance());
        this->angle_motor = new Motor(5);
        this->vertical_motor = new Motor(6);
        this->forward_motor = new Motor(7);
    }

    /// @brief Destruct the object
    ~ArmControl() {
        delete this->angle_motor;
        delete this->vertical_motor;
        delete this->forward_motor;
    }

public:
    /// @brief Stop all arm motors
    /// @note Also resets PID state so a later servo session starts clean.
    void stop() {
        this->angle_motor->stop();
        this->vertical_motor->stop();
        this->forward_motor->stop();
        resetPID();
    }

    /// @brief Stop only servo motors (angle and forward), keep vertical motor state
    /// @note Also resets PID state (integral/derivative) to prevent windup on resume.
    ///       Vertical motor is speed-controlled and not affected by resetPID().
    void stopServo() {
        this->angle_motor->stop();
        this->forward_motor->stop();
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

        this->angle_motor->set_rotate_speed(angle_rpm);

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

        this->forward_motor->set_rotate_speed(forward_rpm);
    }

    /**
     * @brief Manual control for vertical motor (speed mode)
     * @param rpm Rotation speed, positive = up, negative = down
     */
    void manualVertical(float rpm) {
        this->vertical_motor->set_rotate_speed(rpm);
    }

    /**
     * @brief Set vertical motor to absolute position (RESERVED - not implemented)
     * @param position Target position in degrees or encoder counts
     * @note RESERVED: position-control protocol is not implemented yet. Current
     *       vertical motion uses open-loop speed control (manualVertical). Once a
     *       position command is available on the motor bus, fill in this body.
     */
    void setVerticalPosition(float position) {
        (void)position;  // RESERVED - user to implement position protocol
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
};
