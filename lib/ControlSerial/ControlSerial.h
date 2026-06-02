#pragma once
#include<Arduino.h>
#include<cmath>

/**
 * @brief This is the tool class for control serial
 * @note [ ! ] Only can be created once in the whole program
 * @note == Verison 1.0.1 ==
 */
class ControlSerial{
    private:
        int baud_rate = 0; /// Baud rate
        int rx_pin = 0; /// Pin for RX
        int tx_pin = 0; /// Pin for tx
        int delay_time = 2; /// Delay time after a message is sent
    
    public:
        //Disabled copying and quotation
		ControlSerial(const ControlSerial&) = delete;
        ControlSerial& operator=(const ControlSerial&) = delete;

        /**
         * @brief Create control serial object
         * @param baud_rate Baud rate
         * @param rx_pin Pin for RX
         * @param tx_pin Pin for tx
         */
        ControlSerial(int baud_rate, int rx_pin, int tx_pin){
            this->baud_rate = baud_rate;
            this->rx_pin = rx_pin;
            this->tx_pin = tx_pin;
            Serial2.begin(baud_rate, SERIAL_8N1, rx_pin, tx_pin);
            delay(1000);
        }
    
    public:
        /**
		 * @brief Function for changing the address
         * @param old_address Current address of the motor
		 * @param new_address The address you want to change to
		 * @note [ ! ] Try not to use
		 */
        void change_address(int old_address, int new_address){
            uint8_t change_address_command[] = {
                static_cast<uint8_t>(old_address),
                0xAE,
                0x4B,
                0x01,
                static_cast<uint8_t>(new_address),
                0x6B};
            Serial2.write(change_address_command, 6);
            delay(this->delay_time);
        }

        /**
         * @brief Stop the motor
         * @param address The address of the motor
         */
        void stop(int address){
            uint8_t stop_command[] = {
                static_cast<uint8_t>(address),
                0xFE,
                0x98,
                0x00,
                0x6B};
            Serial2.write(stop_command, 5);
            delay(this->delay_time);
        }

        /**
         * @brief Function for setting rotation speed
         * @param address The address of the motor
         * @param rpm Rotation speed, must be less than 3000
         * @note +rpm -> CW
         * @note [ ! ] "S_Vel_IS" in motor should be enabled
         */
        void set_rpm(int address, float rpm){
            uint8_t direction = 0;
            int final_rpm = static_cast<int>(std::round(10 * rpm));
            if (final_rpm < 0){
                direction = 1;
                final_rpm = -final_rpm;}

            uint8_t speed_command[] = {
                static_cast<uint8_t>(address),
                0xF6,
                direction,
                static_cast<uint8_t>((final_rpm >> 8) & 0xFF),
                static_cast<uint8_t>(final_rpm & 0xFF),
                0x00,
                0x00,
                0x6B};
            Serial2.write(speed_command, 8);
            delay(this->delay_time);
        }

        /**
         * @brief Function for setting absolute position (RESERVED - implementation pending)
         * @param address The address of the motor
         * @param position Target position in degrees or encoder counts
         * @note Protocol command 0xFD for position mode
         * @note [ ! ] This is a RESERVED interface - protocol implementation to be filled by user
         * @note [ ! ] Motor must support position mode and have appropriate settings configured
         */
        void set_position(int address, float position){
            // RESERVED: User to implement position control protocol (command 0xFD)
            // Typical protocol structure for reference:
            // uint8_t position_command[] = {
            //     static_cast<uint8_t>(address),
            //     0xFD,  // Position control command
            //     direction,
            //     position_high_byte,
            //     position_low_byte,
            //     speed_high_byte,
            //     speed_low_byte,
            //     0x6B
            // };
            // Serial2.write(position_command, 8);
            // delay(this->delay_time);

            (void)address;
            (void)position;
            // Placeholder: does nothing until user implements the protocol
        }
};
