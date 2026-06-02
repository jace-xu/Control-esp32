#pragma once
#include<Arduino.h>

#define CONTROLSERIAL_DELAY_TIME_WHILE_CHANGE_SERIAL 1000
#define CONTROLSERIAL_COMMAND_MAX_LENGTH 16
#define CONTROLSERIAL_LONG_COMMAND_MAX_LENGTH 96

/**
 * @brief This is the tool class for control serial
 * @note Only have one instance in the whole program
 * @note Please using "ControlSerial::get_instance()" to get the reference of the only instance
 * @note And using "you_instance_reference.initialize(parameters)" to initialize
 * @note [ ! ] If the instance has not been initialized, it will not send or receive any message
 * @note [ ! ] Always using Serial2, if fail to initialize with Serial2, the program may crash (if Serial2 is not used and the prameters is correct, the probability is extremely low)
 * @note [ ! ] A recursive mutex will be created to protect multi-threading safety, if fail, the program can still run but loss the multi-threading safety
 */
class ControlSerial{
    private:
        int baud_rate = 0; /// Baud rate
        int rx_pin = 0; /// Pin for RX
        int tx_pin = 0; /// Pin for tx

        uint8_t command[CONTROLSERIAL_COMMAND_MAX_LENGTH]; // Temporary command
        int command_length = 0; /// Length for the command
        uint8_t long_command[CONTROLSERIAL_LONG_COMMAND_MAX_LENGTH] = {0x00, 0xAA, 0x00, 0x00}; /// Temporary long command
        int long_command_length = 4; /// Length for the long command

        SemaphoreHandle_t mutex; /// Recursive mutex for protecting multi-threading safety
        bool initialized = false; /// Whether the instance be initialized or not
        
        ControlSerial(){this->mutex = xSemaphoreCreateRecursiveMutex();} /// Create recursive mutex
        ~ControlSerial(){if (this->mutex){vSemaphoreDelete(this->mutex);}} /// Delete recursive mutex while destrust the instance
    
    public: // Functions for protecting multi-threading
        /**
         * @brief Lock the usage of the control serial in current thread
         * @note If you want to perform a series of operations in a thread, please thread_lock(), and perform you operations, and finally thread_unlock()
         * @note [ ! ] "thread_lock()" and "thread_unlock()" must be used in pairs
         * @note [ ! ] If you thread_lock() but do not thread_unlock(), other threads will be stopped forever
         * @note [ ! ] If you thread_unlock() without thread_lock() before, the program may crash
         */
        void thread_lock(){if (this->mutex){xSemaphoreTakeRecursive(this->mutex, portMAX_DELAY);}}

        /**
         * @brief Unlock the usage of the control serial in current thread
         * @note If you want to perform a series of operations in a thread, please thread_lock(), and perform you operations, and finally thread_unlock()
         * @note [ ! ] "thread_lock()" and "thread_unlock()" must be used in pairs
         * @note [ ! ] If you thread_lock() but do not thread_unlock(), other threads will be stopped forever
         * @note [ ! ] If you thread_unlock() without thread_lock() before, the program may crash
         */
        void thread_unlock(){if (this->mutex){xSemaphoreGiveRecursive(this->mutex);}}
    
    public: // Initialization
        //Disabled copying
		ControlSerial(const ControlSerial&) = delete;
        ControlSerial& operator=(const ControlSerial&) = delete;

        /// @brief Get the reference of the only instance
        /// @return Reference of the only instance
        static ControlSerial& get_instance(){
            static ControlSerial instance;
            return instance;
        }

        /**
         * @brief Initializing or changing the control serial instance, using Serial2
         * @param baud_rate Baud rate
         * @param rx_pin Pin for RX
         * @param tx_pin Pin for tx
         */
        void initialize(int baud_rate, int rx_pin, int tx_pin){
            this->thread_lock();
            if (this->initialized){
                Serial2.end();
                delay(CONTROLSERIAL_DELAY_TIME_WHILE_CHANGE_SERIAL);
            }
            this->baud_rate = baud_rate;
            this->rx_pin = rx_pin;
            this->tx_pin = tx_pin;
            Serial2.begin(baud_rate, SERIAL_8N1, rx_pin, tx_pin);
            this->initialized = true;
            delay(CONTROLSERIAL_DELAY_TIME_WHILE_CHANGE_SERIAL);
            this->thread_unlock();
        }

        /**
         * @brief Release the usage of Serial2
         */
        void release(){
            this->thread_lock();
            if (this->initialized){
                Serial2.end();
                this->initialized = false;
                delay(CONTROLSERIAL_DELAY_TIME_WHILE_CHANGE_SERIAL);
            }
            this->thread_unlock();
        }
    
    public: // Functions for command operation
        /**
         * @brief Clearing the command you buffered before
         */
        void clear_command(){
            this->thread_lock();
            if (!this->initialized){
                this->thread_unlock();
                return;
            }
            this->command_length = 0;
            this->thread_unlock();
        }

        /**
         * @brief Sending the command you buffered before
         * @note If you send the command successfully, the command will be clear
         * @note If you didn't generate a command after last sending, nothing will be sent
         */
        void send_command(){
            this->thread_lock();
            if ((!this->initialized) || (this->command_length == 0)){
                this->thread_unlock();
                return;
            }
            Serial2.write(this->command, this->command_length);
            this->command_length = 0;
            Serial2.flush();
            this->thread_unlock();
        }

        /**
         * @brief Clearing the long command you buffered before
         */
        void clear_long_command(){
            this->thread_lock();
            if (!this->initialized){
                this->thread_unlock();
                return;
            }
            this->long_command_length = 4;
            this->thread_unlock();
        }

        /**
         * @brief Appending the buffered command to the long command
         * @note If you append the command successfully, the command will be clear
         * @note If the long command reaches its max length, the command will not be appended
         */
        void append_command(){
            this->thread_lock();
            if ((!this->initialized) || (this->command_length == 0) || (this->command_length + this->long_command_length + 1> CONTROLSERIAL_LONG_COMMAND_MAX_LENGTH)){
                this->thread_unlock();
                return;
            }
            for (int byte = 0; byte < this->command_length; byte++){this->long_command[this->long_command_length++] = this->command[byte];}
            this->command_length = 0;
            this->thread_unlock();
        }

        /**
         * @brief Sending the long command you buffered before
         * @note If you send the long command successfully, the long command will be clear
         * @note If you didn't buffer a long_command after last sending, nothing will be sent
         */
        void send_long_command(){
            this->thread_lock();
            if ((!this->initialized) || (this->long_command_length == 4)){
                this->thread_unlock();
                return;
            }
            this->long_command[this->long_command_length++] = 0x6B;
            this->long_command[2] = (this->long_command_length >> 8) & 0xFF;
            this->long_command[3] = this->long_command_length & 0xFF;
            Serial2.write(this->long_command, this->long_command_length);
            this->long_command_length = 4;
            Serial2.flush();
            this->thread_unlock();
        }
    
    public: // Functions for generating and buffering commands
        /**
		 * @brief Generating and buffering the command of changing the address
         * @param old_address Current address of the motor (0 ~ 255)
		 * @param new_address The address you want to change to (0 ~ 255)
         * @param save Whether save the change after rebooting or not
         * @note If there is a command generated before, the now command will cover the old one
		 * @note [ ! ] Try not to use
		 */
        void generate_change_address_command(int old_address, int new_address, bool save){
            this->thread_lock();
            if (!this->initialized){
                this->thread_unlock();
                return;
            }
            this->command[0] = old_address;
            this->command[1] = 0xAE;
            this->command[2] = 0x4B;
            this->command[3] = save;
            this->command[4] = new_address;
            this->command[5] = 0x6B;
            this->command_length = 6;
            this->thread_unlock();
        }

        /**
         * @brief Generating and buffering the command of stopping the motor
         * @param address The address of the motor you want to control (0 ~ 255)
         * @param sync Whether execute the command synchronously or not
         * @note If there is a command generated before, the now command will cover the old one
         */
        void generate_stop_command(int address, bool sync){
            this->thread_lock();
            if (!this->initialized){
                this->thread_unlock();
                return;
            }
            this->command[0] = address;
            this->command[1] = 0xFE;
            this->command[2] = 0x98;
            this->command[3] = sync;
            this->command[4] = 0x6B;
            this->command_length = 5;
            this->thread_unlock();
        }

        /**
         * @brief Generating and buffering the command of setting rotation speed
         * @param address The address of the motor you want to control (0 ~ 255)
         * @param rotate_speed Rotation speed in rpm (0 ~ 3000)
         * @param acceleration Acceleration grade of changing rotation speed (0 ~ 255)
         * @param sync Whether execute the command synchronously or not
         * @note +rotate_speed -> CW
         * @note "acceleration" is not real
         * @note If there is a command generated before, the now command will cover the old one
         * @note [ ! ] "S_Vel_IS" in motor should be enabled
         */
        void generate_set_rotate_speed_command(int address, float rotate_speed, int acceleration, bool sync){
            bool direction = false;
            int final_rotate_speed = static_cast<int>(std::round(10 * rotate_speed));
            if (final_rotate_speed < 0){
                direction = true;
                final_rotate_speed = -final_rotate_speed;}

            this->thread_lock();
            if (!this->initialized){
                this->thread_unlock();
                return;
            }
            this->command[0] = address;
            this->command[1] = 0xF6;
            this->command[2] = direction;
            this->command[3] = (final_rotate_speed >> 8) & 0xFF;
            this->command[4] = final_rotate_speed & 0xFF;
            this->command[5] = acceleration;
            this->command[6] = sync;
            this->command[7] = 0x6B;
            this->command_length = 8;
            this->thread_unlock();
        }
};