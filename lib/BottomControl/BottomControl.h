#pragma once
#include <Motor.h>
#include <array>

/**
 * @brief This is the tool class for controlling bottom
 * @note Direction: Looking down: +X -> front, +Y -> left, +w -> CCW
 * @note [ ! ] Address 1 ~ 4 are used in controlling bottom
 * @note [ ! ] Only can be created once in the whole program
 * @note == Verison 1.0.1 ==
 */
class BottomControl{
    private:
        ControlSerial* control_serial = nullptr; /// Point to control serial object
        Motor* front_left = nullptr; /// Point to front left motor object
        Motor* front_right = nullptr; /// Point to front right motor object
        Motor* back_left = nullptr; /// Point to back left motor object
        Motor* back_right = nullptr; /// Point to back right motor object
        float wheel_radius = 30; /// Wheel radius, in millimeter
        float l_x = 107; /// Half distance in X direction between wheel center to car center, in millimeter
        float l_y = 128.5; /// Half distance in Y direction between wheel center to car center, in millimeter
        float pi = 3.1415926; /// Pi in math
    
    public:
        //Disabled copying and quotation
        BottomControl(const BottomControl&) = delete;
        BottomControl& operator=(const BottomControl&) = delete;
    
        /**
         * @brief Create bottom control object
         * @param serial The serial for controlling motors
         */
        BottomControl(ControlSerial& serial){
            this->control_serial = &serial;
            this->front_left = new Motor(serial, 1);
            this->front_right = new Motor(serial, 2);
            this->back_left = new Motor(serial, 3);
            this->back_right = new Motor(serial, 4);
        }

        /// @brief Destruct the object
        ~BottomControl(){
            delete this->front_left;
            delete this->front_right;
            delete this->back_left;
            delete this->back_right;
        }
    
    public:
        /// @brief Stop all the motors in bottom
        void stop(){
            this->front_left->stop();
            this->front_right->stop();
            this->back_left->stop();
            this->back_right->stop();
        }

        /**
         * @brief Core function for controlling motors
         * @param velocities Float array including 3 velocity components
         * @note velocities[0] -> velocity in x direction in millimeter per second
         * @note velocities[1] -> velocity in y direction in millimeter per second
         * @note velocities[2] -> angular velocity in rad per second
         */
        void motors_control(const std::array<float, 3>& velocities){
            float front_left_rpm = (velocities[0] + velocities[1] - (l_x + l_y) * velocities[2]) * 30 / this->pi / this->wheel_radius;
            float front_right_rpm = -(velocities[0] - velocities[1] + (l_x + l_y) * velocities[2]) * 30 / this->pi / this->wheel_radius;
            float back_left_rpm = (velocities[0] - velocities[1] - (l_x + l_y) * velocities[2]) * 30 / this->pi / this->wheel_radius;
            float back_right_rpm = -(velocities[0] + velocities[1] + (l_x + l_y) * velocities[2]) * 30 / this->pi / this->wheel_radius;

            this->front_left->set_rpm(front_left_rpm);
            this->front_right->set_rpm(front_right_rpm);
            this->back_left->set_rpm(back_left_rpm);
            this->back_right->set_rpm(back_right_rpm);
        }
};
