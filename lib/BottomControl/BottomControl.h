#pragma once
#include<ControlSerial.h>

#define BOTTOMCONTROL_WHEEL_RADIUS 30 /// Wheel radius, in millimeter
#define BOTTOMCONTROL_LX 107.0f /// Half distance in X direction between wheel center to car center, in millimeter
#define BOTTOMCONTROL_LY 128.5f /// Half distance in Y direction between wheel center to car center, in millimeter
#define BOTTOMCONTROL_PI 3.1415926f /// Pi in math

/**
 * @brief This is the tool class for controlling bottom
 * @note Direction: Looking down: +X -> front, +Y -> left, +w -> CCW
 * @note Address 1 ~ 4 are used in controlling bottom
 * @note Only have one instance in the whole program
 * @note Please using "BottomControl::get_instance()" to get the reference of the only instance
 * @note [ ! ] If the control serial has not been initialized, it will not send or receive any message
 * @note [ ! ] Always using Serial2, if fail to initialize with Serial2, the program may crash (if Serial2 is not used and the prameters is correct, the probability is extremely low)
 * @note [ ! ] A recursive mutex will be created to protect multi-threading safety, if fail, the program can still run but loss the multi-threading safety
 */
class BottomControl{
    private:
        ControlSerial* control_serial = nullptr; /// Point to control serial object
    
    public: // Initialization
        //Disabled copying and quotation
        BottomControl(const BottomControl&) = delete;
        BottomControl& operator=(const BottomControl&) = delete;
    
        /// @brief Create bottom control instance
        BottomControl(){this->control_serial = &(ControlSerial::get_instance());}

        /// @brief Get the reference of the only instance
        /// @return Reference of the only instance
        static BottomControl& get_instance(){
            static BottomControl instance;
            return instance;
        }
    
    public: // Functions for controling the bottom directly
        /**
         * @brief Stopping all the motors in bottom
         * @note The command you buffered before will be cleared
         */
        void stop(){
            this->control_serial->thread_lock();
            this->control_serial->clear_long_command();
            this->control_serial->generate_stop_command(1);
            this->control_serial->append_command();
            this->control_serial->generate_stop_command(2);
            this->control_serial->append_command();
            this->control_serial->generate_stop_command(3);
            this->control_serial->append_command();
            this->control_serial->generate_stop_command(4);
            this->control_serial->append_command();
            this->control_serial->send_long_command();
            this->control_serial->thread_unlock();
        }

        /**
         * @brief Controlling bottom movement with velocity parameters
         * @param velocities Float array including 3 velocity components
         * @note velocities[0] -> velocity in x direction in millimeter per second
         * @note velocities[1] -> velocity in y direction in millimeter per second
         * @note velocities[2] -> angular velocity in rad per second
         * @note The command you buffered before will be cleared
         */
        void motors_control(const std::array<float, 3>& velocities){
            float rotate_speeds[4];
            rotate_speeds[0] = +(velocities[0] - velocities[1] - (BOTTOMCONTROL_LX + BOTTOMCONTROL_LY) * velocities[2]) * 30 / BOTTOMCONTROL_PI / BOTTOMCONTROL_WHEEL_RADIUS;
            rotate_speeds[1] = -(velocities[0] + velocities[1] + (BOTTOMCONTROL_LX + BOTTOMCONTROL_LY) * velocities[2]) * 30 / BOTTOMCONTROL_PI / BOTTOMCONTROL_WHEEL_RADIUS;
            rotate_speeds[2] = +(velocities[0] + velocities[1] - (BOTTOMCONTROL_LX + BOTTOMCONTROL_LY) * velocities[2]) * 30 / BOTTOMCONTROL_PI / BOTTOMCONTROL_WHEEL_RADIUS;
            rotate_speeds[3] = -(velocities[0] - velocities[1] + (BOTTOMCONTROL_LX + BOTTOMCONTROL_LY) * velocities[2]) * 30 / BOTTOMCONTROL_PI / BOTTOMCONTROL_WHEEL_RADIUS;
            this->control_serial->thread_lock();
            this->control_serial->clear_long_command();
            this->control_serial->Emm_generate_set_rotate_speed_command(1, rotate_speeds[0]);
            this->control_serial->append_command();
            this->control_serial->Emm_generate_set_rotate_speed_command(2, rotate_speeds[1]);
            this->control_serial->append_command();
            this->control_serial->Emm_generate_set_rotate_speed_command(3, rotate_speeds[2]);
            this->control_serial->append_command();
            this->control_serial->Emm_generate_set_rotate_speed_command(4, rotate_speeds[3]);
            this->control_serial->append_command();
            this->control_serial->send_long_command();
            this->control_serial->thread_unlock();
        }
    
    public: // Functions for generating and buffering commands
        /**
         * @brief Generating and buffering the command of stopping all the motors in bottom
         * @note The command you buffered before will be cleared and the new commands (4 motor stop) will be appended
         */
        void append_stop_command(){
            this->control_serial->thread_lock();
            this->control_serial->clear_long_command();
            this->control_serial->generate_stop_command(1);
            this->control_serial->append_command();
            this->control_serial->generate_stop_command(2);
            this->control_serial->append_command();
            this->control_serial->generate_stop_command(3);
            this->control_serial->append_command();
            this->control_serial->generate_stop_command(4);
            this->control_serial->append_command();
            this->control_serial->thread_unlock();
        }

        /**
         * @brief Generating and buffering the command of controlling bottom movement with velocity parameters
         * @param velocities Float array including 3 velocity components
         * @note velocities[0] -> velocity in x direction in millimeter per second
         * @note velocities[1] -> velocity in y direction in millimeter per second
         * @note velocities[2] -> angular velocity in rad per second
         * @note The command you buffered before will be cleared
         */
        void append_motors_control_command(const std::array<float, 3>& velocities){
            float rotate_speeds[4];
            rotate_speeds[0] = +(velocities[0] + velocities[1] - (BOTTOMCONTROL_LX + BOTTOMCONTROL_LY) * velocities[2]) * 30 / BOTTOMCONTROL_PI / BOTTOMCONTROL_WHEEL_RADIUS;
            rotate_speeds[1] = -(velocities[0] - velocities[1] + (BOTTOMCONTROL_LX + BOTTOMCONTROL_LY) * velocities[2]) * 30 / BOTTOMCONTROL_PI / BOTTOMCONTROL_WHEEL_RADIUS;
            rotate_speeds[2] = +(velocities[0] - velocities[1] - (BOTTOMCONTROL_LX + BOTTOMCONTROL_LY) * velocities[2]) * 30 / BOTTOMCONTROL_PI / BOTTOMCONTROL_WHEEL_RADIUS;
            rotate_speeds[3] = -(velocities[0] + velocities[1] + (BOTTOMCONTROL_LX + BOTTOMCONTROL_LY) * velocities[2]) * 30 / BOTTOMCONTROL_PI / BOTTOMCONTROL_WHEEL_RADIUS;
            this->control_serial->thread_lock();
            this->control_serial->clear_long_command();
            this->control_serial->Emm_generate_set_rotate_speed_command(1, rotate_speeds[0]);
            this->control_serial->append_command();
            this->control_serial->Emm_generate_set_rotate_speed_command(2, rotate_speeds[1]);
            this->control_serial->append_command();
            this->control_serial->Emm_generate_set_rotate_speed_command(3, rotate_speeds[2]);
            this->control_serial->append_command();
            this->control_serial->Emm_generate_set_rotate_speed_command(4, rotate_speeds[3]);
            this->control_serial->append_command();
            this->control_serial->thread_unlock();
        }
};