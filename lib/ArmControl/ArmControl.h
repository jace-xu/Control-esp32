#pragma once
#include <ControlSerial.h>
#include <Arduino.h>
#include <cmath>
#include <VisionSerial.h>
#include <VisionStreamHandshake.h>

/**
 * @brief Arm control class for visual servoing and manual control
 * @note Manages 3 motors via ControlSerial: angle (addr 5), vertical (addr 6), forward (addr 7)
 * @note Addresses 5-7 share the same Serial2 bus with chassis motors (addresses 1-4)
 * @note 夹爪由 ESP32 LEDC PWM 驱动一个舵机 (grip/release 到固定角度)
 * @note [ ! ] Only can be created once in the whole program
 * @note 集成视觉对准: 持有 VisionStreamHandshake, 提供角度+前后 PID 闭环对准 +
 *       串流握手 + 收敛判定 (beginAlign / updateAlign / isAligned), 供上层序列复用。
 * @note == Version 3.0.0 ==
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
    static constexpr float kGripperClosedAngle = 120.0f;   // 夹紧角度 (度)
    static constexpr float kGripperOpenAngle = 180.0f;    // 松开角度 (度)
    // 舵机脉宽范围 (微秒): 多数舵机 0.5ms~2.5ms 对应 0~180 度
    static constexpr float kGripperMinPulseUs = 500.0f;
    static constexpr float kGripperMaxPulseUs = 2500.0f;

    ControlSerial* control_serial = nullptr;  // 共用的 Serial2 命令通道 (单例)

    // ---- 视觉对准 (集成) ----
    VisionSerial* vision = nullptr;       // 视觉数据串口 (接收树莓派误差; 不拥有生命周期)
    VisionStreamHandshake* handshake = nullptr;  // 视觉串流握手会话 (START/STOP + 超时重传)

    // PID 收敛判定 (对准完成后才算"已对准")
    static constexpr uint8_t kConvergeFramesNeeded = 10;   // 连续入阈帧数 (50Hz ≈ 0.2s)
    static constexpr float kConvergeAngleThresh = 8.0f;    // 角度轴误差收敛阈值 (像素)
    static constexpr float kConvergeForwardThresh = 8.0f;  // 前后轴误差收敛阈值 (像素)

    uint8_t converge_count = 0;       // PID 收敛连续入阈帧计数
    bool timeout_warned = false;      // "无有效视觉数据" 警告是否已打印过
    uint32_t last_debug_print = 0;    // 上次打印视觉误差调试信息的时间戳
    bool count_convergence = true;    // 是否在本次会话累计收敛 (对准期 true)
    bool servo_idle_stopped = false;  // 无数据期是否已停过一次伺服 (防每帧重发停止命令灌总线)


    // PID controller parameters (tunable)
    // [ ! ] 这里是唯一权威来源 (现场标定值)。运行时如需改, 调 setAnglePID/setForwardPID。
    float kp_angle = 0.02f;      // P gain for angle servo
    float ki_angle = 0.0f;        // I gain for angle servo (积分项)
    float kd_angle = 0.003f;      // D gain for angle servo (微分项)

    float kp_forward = 0.2f;     // P gain for forward servo
    float ki_forward = 0.0f;      // I gain for forward servo
    float kd_forward = 0.003f;    // D gain for forward servo

    float deadzone = 5.0f;        // Error deadzone threshold
    float max_rpm = 50.0f;       // Maximum motor speed limit

    // 伺服方向因子: PID 算出的 rpm 在下发前乘以此符号, 用于翻转电机转向。
    // [ ! ] 与 PID 增益解耦: 调"方向"只翻这里的 +1/-1, 调"力度"才动 kp/ki/kd。
    //       联调判据: 给固定正误差, 若电机往"误差变大"方向转(发散), 把对应轴符号取反。
    static constexpr float kAngleServoSign = -1.0f;    // 角度轴转向: +1 / -1
    static constexpr float kForwardServoSign = 1.0f;  // 前后轴转向: +1 / -1

    // PID state variables (internal)
    float angle_error_integral = 0.0f;     // 角度误差积分
    float angle_error_last = 0.0f;         // 上一次角度误差(用于微分)
    float forward_error_integral = 0.0f;   // 前后误差积分
    float forward_error_last = 0.0f;       // 上一次前后误差

    uint32_t last_update_time = 0;         // 上次更新时间(用于计算dt)

    // Anti-windup limits (防止积分饱和)
    float integral_limit = 1000.0f;        // 积分项限制

    // 电机位置移动速度 (rpm), setPosition 时使用 (按电机区分)
    static constexpr float kVerticalPosSpeed = 800.0f;  // 上下电机位置速度
    static constexpr float kForwardPosSpeed = 120.0f;   // 前后电机位置速度
    static constexpr float kAnglePosSpeed = 15.0f;      // 角度电机位置速度

    /// @brief 按电机地址返回其位置移动速度 (rpm)
    /// @note  地址未知时回退到角度电机速度 (最慢, 最保守)
    static constexpr float posSpeedFor(int address) {
        return address == kVerticalAddr ? kVerticalPosSpeed
             : address == kForwardAddr  ? kForwardPosSpeed
             :                            kAnglePosSpeed;
    }

    // 位置问询后等待电机回复的时间 (ms), readVerticalPosition 时使用
    // 给电机收到问询、处理、回 8 字节的往返留出时间, 否则 read 会提前超时
    static constexpr uint32_t kPositionReadDelayMs = 2;

public:
public:
    /// @brief 视觉对准一帧的结果, 供上层序列按阶段语义处理
    struct AlignFrame {
        bool freshFrame = false;  // 本帧是否收到与请求类型匹配的新鲜视觉帧
        uint8_t color = 0;        // 新鲜帧携带的物块颜色 (id2); freshFrame=false 时无意义
        bool converged = false;   // 当前是否已连续收敛达标 (可确认下一步)
    };

    // Disabled copying and assignment
    ArmControl(const ArmControl&) = delete;
    ArmControl& operator=(const ArmControl&) = delete;

    /**
     * @brief Create arm control object
     * @param vision 已初始化的视觉串口指针 (用于视觉对准; 不拥有其生命周期)
     * @note Motors share the single Serial2 bus via ControlSerial::get_instance()
     * @note 同时初始化夹爪舵机的 LEDC PWM, 上电默认松开
     * @note 内部 new 一个 VisionStreamHandshake (持有 vision 指针), 析构时 delete
     */
    explicit ArmControl(VisionSerial* vision) {
        this->control_serial = &(ControlSerial::get_instance());
        this->vision = vision;
        this->handshake = new VisionStreamHandshake(vision);
        // 初始化夹爪舵机 PWM (ESP32 Arduino core 2.x: 通道制 LEDC)
        ledcSetup(kGripperPwmChannel, kGripperPwmFreq, kGripperPwmResBits);
        ledcAttachPin(kGripperPin, kGripperPwmChannel);
        release();  // 上电默认松开夹爪
    }

    /// @brief Destruct the object
    ~ArmControl() {
        ledcDetachPin(kGripperPin);
        delete handshake;
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
        this->control_serial->clear_long_command();
        this->control_serial->generate_stop_command(kAngleAddr);
        this->control_serial->append_command();
        this->control_serial->generate_stop_command(kVerticalAddr);
        this->control_serial->append_command();
        this->control_serial->generate_stop_command(kForwardAddr);
        this->control_serial->append_command();
        this->control_serial->send_long_command();
        this->control_serial->thread_unlock();
        resetPID();
    }

    /// @brief Stop only servo motors (angle and forward), keep vertical motor state
    /// @note Also resets PID state (integral/derivative) to prevent windup on resume.
    ///       Vertical motor is speed-controlled and not affected by resetPID().
    void stopServo() {
        this->control_serial->thread_lock();
        this->control_serial->clear_long_command();
        this->control_serial->generate_stop_command(kAngleAddr);
        this->control_serial->append_command();
        this->control_serial->generate_stop_command(kForwardAddr);
        this->control_serial->append_command();
        this->control_serial->send_long_command();
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

        // 方向因子: 下发前乘以转向符号 (与增益解耦, 翻方向只改 kAngleServoSign/kForwardServoSign)
        angle_rpm *= kAngleServoSign;
        forward_rpm *= kForwardServoSign;

        // 角度 + 前后 两条速度命令合并为一条长命令一次性下发:
        // 单次总线事务, 两个伺服电机更同步, 也省去逐条 flush 的开销。
        // 上下电机已改用位置控制(设一次), 不在此每帧重发。
        // 全程持锁, 防止 append 序列被其他线程的命令插入而错帧。
        
        this->control_serial->clear_long_command();
        this->control_serial->X_generate_set_rotate_speed_command(kAngleAddr, angle_rpm);
        this->control_serial->append_command();
        this->control_serial->X_generate_set_rotate_speed_command(kForwardAddr, forward_rpm);
        this->control_serial->append_command();
        this->control_serial->send_long_command();

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
        this->control_serial->X_generate_set_position_command(
            address, positionDegrees, posSpeedFor(address));
        this->control_serial->send_command();
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
     * @brief 角度+前后两个电机位置命令打包成一条长指令一次性下发
     * @param angleDegrees   角度电机目标绝对角 (度)
     * @param forwardDegrees 前后电机目标绝对角 (度)
     * @note  两条 0xFD 位置子命令拼进同一个长指令帧 (0xAA 头), 单次 Serial2 写出,
     *        避免两条命令背靠背发送时的帧间隔/丢帧问题。
     * @note  全程持递归锁为原子事务, 防其他线程在拼帧期间插命令。
     * @note  供 PRE_CATCH 预备摆位使用 (角度+前后同时到位)。
     */
    void setAngleForwardPosition(float angleDegrees, float forwardDegrees) {
        this->control_serial->thread_lock();
        this->control_serial->clear_long_command();
        // 角度电机子命令: 构造到 command 缓冲后追加进长指令
        this->control_serial->X_generate_set_position_command(
            kAngleAddr, angleDegrees, posSpeedFor(kAngleAddr));
        this->control_serial->append_command();
        // 前后电机子命令
        this->control_serial->X_generate_set_position_command(
            kForwardAddr, forwardDegrees, posSpeedFor(kForwardAddr));
        this->control_serial->append_command();
        this->control_serial->send_long_command();
        this->control_serial->thread_unlock();
    }

    /**
     * @brief 上下+前后两个电机位置命令打包成一条长指令一次性下发
     * @param verticalDegrees 上下电机目标绝对角 (度)
     * @param forwardDegrees  前后电机目标绝对角 (度)
     * @note  两条 0xFD 位置子命令拼进同一个长指令帧 (0xAA 头), 单次 Serial2 写出,
     *        避免两条命令背靠背发送时的帧间隔/丢帧问题。
     * @note  全程持递归锁为原子事务, 防其他线程在拼帧期间插命令。
     * @note  供阶段1 收臂步骤1 使用 (上下+前后同时归位)。
     */
    void setVerticalForwardPosition(float verticalDegrees, float forwardDegrees) {
        this->control_serial->thread_lock();
        this->control_serial->clear_long_command();
        // 上下电机子命令: 构造到 command 缓冲后追加进长指令
        this->control_serial->X_generate_set_position_command(
            kVerticalAddr, verticalDegrees, posSpeedFor(kVerticalAddr));
        this->control_serial->append_command();
        // 前后电机子命令
        this->control_serial->X_generate_set_position_command(
            kForwardAddr, forwardDegrees, posSpeedFor(kForwardAddr));
        this->control_serial->append_command();
        this->control_serial->send_long_command();
        this->control_serial->thread_unlock();
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

    // ========================================================================
    // 视觉对准 (集成: 角度+前后 PID 闭环对准 + 串流握手 + 收敛判定)
    // 供上层序列 (阶段1 夹取 / 阶段2 放置) 复用; 只管角度(5)+前后(7), 不碰上下(6)。
    // ========================================================================

    /**
     * @brief 开始一次对准会话: 请求串流 + 复位 PID + 清零收敛计数
     * @param errorType   误差类型: 1=误差1, 2=误差2(按颜色)
     * @param color       误差2 的目标颜色; 误差1 传 kColorNone
     * @param currentTime 当前 millis() 时间戳
     */
    void beginAlign(uint8_t errorType, uint8_t color, uint32_t currentTime) {
        handshake->start(errorType, color, currentTime);
        resetPID();
        converge_count = 0;
        timeout_warned = false;
        count_convergence = true;
        servo_idle_stopped = false;   // 新会话: 重置断流停止守卫
    }

    /// @brief 是否累计收敛 (对准期 true; 转位置控制后调用方置 false)
    void setCountConvergence(bool on) { count_convergence = on; }

    /// @brief 当前是否已请求串流 (供调用方判断是否进伺服分支)
    bool isStreaming() const { return handshake->isStreaming(); }

    /// @brief 当前是否已连续收敛达标
    /// @note  [手动确认模式] 收敛判定已注释 (converge_count 不再累加, 恒为 0),
    ///        故此处无条件返回 true: 第二次按 A 即视为"已对准", 直接确认下降。
    ///        若要恢复自动收敛: 取消 updateAlign 里收敛计数块的注释, 并改回
    ///        return converge_count >= kConvergeFramesNeeded;
    bool isAligned() const { return true; }

    /// @brief 本次串流请求的误差类型 (1/2)
    uint8_t expectedErrorType() const { return handshake->expectedErrorType(); }

    /// @brief START 命令超时重传检查 (转发握手会话)
    void retryAlignIfNeeded(uint32_t currentTime) { handshake->retryIfNeeded(currentTime); }

    /// @brief 通知树莓派停止发送误差数据 (转发握手会话)
    void stopAlignStream() { handshake->stop(); }

    /**
     * @brief 视觉伺服一帧: 读帧 → 校验类型 → PID 伺服 → 收敛计数
     * @param currentTime 当前 millis() 时间戳
     * @return AlignFrame: 本帧是否新鲜 / 颜色 / 是否已收敛
     * @note  不记色入队、不打印 "[Stage X]" 提示, 交调用方按 AlignFrame 自行处理。
     */
    AlignFrame updateAlign(uint32_t currentTime) {
        AlignFrame result;
        VisionError visionError = vision->read();

        // 仅接受与本次请求误差类型匹配的帧 (id1: 1=误差1, 2=误差2)。
        // 切换串流 (STOP→START) 瞬间缓冲可能残留旧帧, 不校验会误判握手/记错色/用旧误差。
        bool frameMatches = visionError.valid && (visionError.id1 == handshake->expectedErrorType());

        if (frameMatches) {
            handshake->confirm();   // 类型匹配: 确认握手成功 (停止 START 重传)
            timeout_warned = false;
            servo_idle_stopped = false;   // 数据恢复: 重新允许下次断流时再停一次
            result.freshFrame = true;
            result.color = visionError.id2;

            updateVisualServo(visionError.angleError, visionError.forwardError);

            // 收敛判定: 角度与前后误差同时入阈才计数, 任一超阈值则清零。
            // if (count_convergence) {
            //     if (fabsf(visionError.angleError) <= kConvergeAngleThresh &&
            //         fabsf(visionError.forwardError) <= kConvergeForwardThresh) {
            //         if (converge_count < kConvergeFramesNeeded) {
            //             converge_count++;
            //         }
            //     } else {
            //         converge_count = 0;
            //     }
            // }

            // 调试输出: 每 200ms 打印一次视觉误差值
            if ((currentTime - last_debug_print) > 200) {
                Serial.printf("Vision: angle=%.2f, forward=%.2f conv=%u/%u\n",
                              visionError.angleError, visionError.forwardError,
                              converge_count, kConvergeFramesNeeded);
                last_debug_print = currentTime;
            }
        } else {
            // 无有效帧: 停伺服并复位 PID, 防数据恢复瞬间积分饱和窜动。
            // [ ! ] 只在"由有数据→无数据"的边沿停一次, 不每帧重发: 每帧停止命令会持续
            //       占用 Serial2 总线, 干扰同总线上下电机(6)的位置命令时序。
            if (!servo_idle_stopped) {
                stopServo();
                servo_idle_stopped = true;
            }
            if (!timeout_warned) {
                Serial.println("WARNING: No valid vision data");
                timeout_warned = true;
            }
        }

        result.converged = isAligned();
        return result;
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
