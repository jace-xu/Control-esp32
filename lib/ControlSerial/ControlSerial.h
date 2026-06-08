#pragma once
#include<Arduino.h>

#define CONTROLSERIAL_DELAY_TIME_WHILE_CHANGE_SERIAL 1000
#define CONTROLSERIAL_RECEIVE_WAIT_TIME 1
#define CONTROLSERIAL_COMMAND_MAX_LENGTH 16
#define CONTROLSERIAL_LONG_COMMAND_MAX_LENGTH 128

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
            if ((!this->initialized) || (!this->command_length)){
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
            if ((!this->initialized) || (!this->command_length) || (this->command_length + this->long_command_length + 1> CONTROLSERIAL_LONG_COMMAND_MAX_LENGTH)){
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
    
    public: // Functions for getting information
        /**
         * @brief Clearing the data in hardware
         * @note If some data is sending by motor in current, the function will wait and clear all data which is sent now
         */
        void clear_received_data(){
            this->thread_lock();
            if (!this->initialized){
                this->thread_unlock();
                return;
            }
            unsigned long time = millis();
            while(1){
                if (Serial2.available()){
                    Serial2.read();
                    time = millis();
                }
                else{if (static_cast<int>(millis() - time) > CONTROLSERIAL_RECEIVE_WAIT_TIME){break;}}
            }
            this->thread_unlock();
        }

        /**
         * @brief (X) Reading the current position data buffered in hardware
         * @param current_position A reference of a float variable, in degree
         * @return Whether successfully read data or not
         * @note +current_position -> CW (looking outward), means the absolute angle position from 0 point (the place when the motor booting)
         * @note The function will change the variable as the current position
         * @note If fail to read data, the variable will not be changed
         * @note If the data you received is invalid, all the data in hardware will be cleared
         */
        bool X_read_current_position(float& current_position){
            this->thread_lock();
            if (!this->initialized){
                this->thread_unlock();
                return false;
            }
            uint8_t current_position_data[8];
            uint8_t current_position_byte = 0;
            unsigned long time = millis();
            while(current_position_byte < 8){
                if (Serial2.available()){
                    current_position_data[current_position_byte++] = Serial2.read();
                    time = millis();
                }
                else{if (static_cast<int>(millis() - time) > CONTROLSERIAL_RECEIVE_WAIT_TIME){break;}}
            }
            if (current_position_byte == 8){
                if (current_position_data[1] == 0x36){
                    if (current_position_data[7] == 0x6B){
                        current_position = static_cast<float>((current_position_data[3] << 24) | (current_position_data[4] << 16) | (current_position_data[5] << 8) | (current_position_data[6])) / 10;
                        if (current_position_data[2]){current_position = -current_position;}
                        this->thread_unlock();
                        return true;
                    }
                }
            }
            this->clear_received_data();
            this->thread_unlock();
            return false;
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
        void generate_change_address_command(int old_address, int new_address, bool save=true){
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
        void generate_stop_command(int address, bool sync=false){
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
         * @brief (Emm) Generating and buffering the command of setting rotation speed
         * @param address The address of the motor you want to control (0 ~ 255)
         * @param rotate_speed Rotation speed in rpm (-3000 ~ 3000)
         * @param acceleration Acceleration grade of changing rotation speed (0 ~ 255)
         * @param sync Whether execute the command synchronously or not
         * @note +rotate_speed -> CW (looking outawrd)
         * @note "acceleration" is not real
         * @note If there is a command generated before, the now command will cover the old one
         * @note [ ! ] "S_Vel_IS" in motor should be enabled
         */
        void Emm_generate_set_rotate_speed_command(int address, float rotate_speed, int acceleration=0, bool sync=false){
            bool direction = false;
            int final_rotate_speed = static_cast<int>(std::round(10 * rotate_speed));
            if (final_rotate_speed < 0){
                direction = true;
                final_rotate_speed = -final_rotate_speed;
            }
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
    
        /**
         * @brief (X) Generating and buffering the command of setting rotation speed
         * @param address The address of the motor you want to control (0 ~ 255)
         * @param rotate_speed Rotation speed in rpm (-3000 ~ 3000)
         * @param acceleration Acceleration value in rpm/s (1 ~ 65535)
         * @param sync Whether execute the command synchronously or not
         * @note +rotate_speed -> CW (looking outawrd)
         * @note If there is a command generated before, the now command will cover the old one
         */
        void X_generate_set_rotate_speed_command(int address, float rotate_speed, int acceleration=65535, bool sync=false){
            bool direction = false;
            int final_rotate_speed = static_cast<int>(std::round(10 * rotate_speed));
            if (final_rotate_speed < 0){
                direction = true;
                final_rotate_speed = -final_rotate_speed;
            }
            this->thread_lock();
            if (!this->initialized){
                this->thread_unlock();
                return;
            }
            this->command[0] = address;
            this->command[1] = 0xF6;
            this->command[2] = direction;
            this->command[3] = (acceleration >> 8) & 0xFF;
            this->command[4] = acceleration & 0xFF;
            this->command[5] = (final_rotate_speed >> 8) & 0xFF;
            this->command[6] = final_rotate_speed & 0xFF;
            this->command[7] = sync;
            this->command[8] = 0x6B;
            this->command_length = 9;
            this->thread_unlock();
        }

        /**
         * @brief (X) Generating and buffering the command of setting position
         * @param address The address of the motor you want to control (0 ~ 255)
         * @param position The absolute position you want the motor to move to in degree (-429496729.5 ~ 429496729.5)
         * @param rotate_speed The max rotation speed when the motor move in rpm (0.1 ~ 3000)
         * @param acceleration_init Acceleration value when starting to move in rpm/s (1 ~ 65535)
         * @param acceleration_final Acceleration value when stopping in rpm/s (1 ~ 65535)
         * @param sync Whether execute the command synchronously or not
         * @note +position -> CW (looking outawrd), means the absolute angle position from 0 point (the place when the motor booting)
         * @note If there is a command generated before, the now command will cover the old one
         */
        void X_generate_set_position_command(int address, float position, float rotate_speed, int acceleration_init=65535, int acceleration_final=65535, bool sync=false){
            bool direction = false;
            int final_position = static_cast<int>(std::round(10 * position));
            if (final_position < 0){
                direction = true;
                final_position = -final_position;
            }
            int final_rotate_speed = static_cast<int>(std::round(10 * rotate_speed));
            this->thread_lock();
            if (!this->initialized){
                this->thread_unlock();
                return;
            }
            this->command[0] = address;
            this->command[1] = 0xFD;
            this->command[2] = direction;
            this->command[3] = (acceleration_init >> 8) & 0xFF;
            this->command[4] = acceleration_init & 0xFF;
            this->command[5] = (acceleration_final >> 8) & 0xFF;
            this->command[6] = acceleration_final & 0xFF;
            this->command[7] = (final_rotate_speed >> 8) & 0xFF;
            this->command[8] = final_rotate_speed & 0xFF;
            this->command[9] = (final_position >> 24) & 0xFF;
            this->command[10] = (final_position >> 16) & 0xFF;
            this->command[11] = (final_position >> 8) & 0xFF;
            this->command[12] = final_position & 0xFF;
            this->command[13] = 0x01;
            this->command[14] = sync;
            this->command[15] = 0x6B;
            this->command_length = 16;
            this->thread_unlock();
        }
    
    public: // Other functions
        /**
         * @brief Asking current position of the motor
         * @param address The address of the motor you want to control (1 ~ 255)
         * @note After asking, you should wait a moment and then read
         * @note It will directly send the command of asking current position
         * @note [ ! ] "Response" in all motors should be "None", or the received data may be invalid
         */
        void ask_current_position(int address){
            this->thread_lock();
            if (!this->initialized){
                this->thread_unlock();
                return;
            }
            this->command[0] = address;
            this->command[1] = 0x36;
            this->command[2] = 0x6B;
            this->command_length = 3;
            this->send_command();
            this->thread_unlock();
        }

        /**
         * @brief Start synchronous movement
         * @param address The address of the motor you want to control (1 ~ 255)
         * @note If you use command(sync=true) before, the command will be buffered by the motor
         * @note So when you use this function, the motor will start to execute the command
         * @note It will directly send the command of starting synchronous movement
         */
        void start_sync_move(int address){
            this->thread_lock();
            if (!this->initialized){
                this->thread_unlock();
                return;
            }
            this->command[0] = address;
            this->command[1] = 0xFF;
            this->command[2] = 0x66;
            this->command[3] = 0x6B;
            this->command_length = 4;
            this->send_command();
            this->thread_unlock();
        }
};