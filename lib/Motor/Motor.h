#pragma once
#include<ControlSerial.h>

/**
 * @brief This is the motor class, include motor controls based on serial
 * @note Direction: +rpm -> CW
 * @note [ ! ] If the control serial has not been initialized, it will not send or receive any message
 * @note [ ! ] Always using Serial2, if fail to initialize with Serial2, the program may crash (if Serial2 is not used and the prameters is correct, the probability is extremely low)
 * @note [ ! ] A recursive mutex will be created to protect multi-threading safety, if fail, the program can still run but loss the multi-threading safety
 */
class Motor{
	private:
		ControlSerial* control_serial = nullptr; /// Point to control serial instance
		int address = 0; /// Address for this motor
	
	public: // Initialization
		//Disabled copying
		Motor(const Motor&) = delete;
        Motor& operator=(const Motor&) = delete;
	
		/**
		 * @brief Create motor instance
		 * @param address Address for this motor (0 ~ 255)
		 */
		Motor(int address){
			this->control_serial = &(ControlSerial::get_instance());
			if (address < 0){this->address = 0;}
			else if (address > 255){this->address = 255;}
			else{this->address = address;}
		}
		
	public: // Functions for thread lock and long command operation
		/**
         * @brief Lock the usage of the control serial in current thread
         * @note If you want to perform a series of operations in a thread, please lock(), and perform you operations, and finally unlock()
         * @note [ ! ] "lock()" and "unlock()" must be used in pairs
         * @note [ ! ] If you lock() but do not unlock(), other threads will be stopped forever
         * @note [ ! ] If you unlock() without lock() before, the program may crash
         */
		void thread_lock(){this->control_serial->thread_lock();}

		/**
         * @brief Unlock the usage of the control serial in current thread
         * @note If you want to perform a series of operations in a thread, please lock(), and perform you operations, and finally unlock()
         * @note [ ! ] "lock()" and "unlock()" must be used in pairs
         * @note [ ! ] If you lock() but do not unlock(), other threads will be stopped forever
         * @note [ ! ] If you unlock() without lock() before, the program may crash
         */
		void thread_unlock(){this->control_serial->thread_unlock();}

		/**
		 * @brief Clearing the long command you buffered before
		 */
		void clear_long_command(){this->control_serial->clear_long_command();}

		/**
		 * @brief Sending the long command you buffered before
		 * @note If you send the long command successfully, the long command will be clear
		 * @note If you didn't buffer a long_command after last sending, nothing will be sent
		 */
		void send_long_command(){this->control_serial->send_long_command();}

	public: // Functions for controling the motor directly
		/**
		 * @brief Changing the address
		 * @param new_address The address you want to change to (0 ~ 255)
		 * @param save Whether save the change after rebooting or not
		 * @note If the new adress is invalid, the function will return
		 * @note [ ! ] If the command is not sent finally, the address buffered in the instance will still change, which may have some mistakes
		 * @note [ ! ] Try not to use
		 */
		void change_address(int new_address, bool save=true){
			if (new_address < 0 || new_address > 255){return;}
			this->control_serial->thread_lock();
			this->control_serial->generate_change_address_command(this->address, new_address, save);
			this->control_serial->send_command();
			this->control_serial->thread_unlock();
			this->address = new_address;
		}

		/**
         * @brief Stopping the motor
         * @param sync Whether execute the command synchronously or not
         */
		void stop(bool sync=false){
			this->control_serial->thread_lock();
			this->control_serial->generate_stop_command(this->address, sync);
			this->control_serial->send_command();
			this->control_serial->thread_unlock();
		}
		
		/**
         * @brief Setting rotation speed
         * @param rotate_speed Rotation speed in rpm (0 ~ 3000)
         * @param acceleration Acceleration grade of changing rotation speed (0 ~ 255)
         * @param sync Whether execute the command synchronously or not
         * @note +rotate_speed -> CW
         * @note "acceleration" is not real
         * @note [ ! ] "S_Vel_IS" in motor should be enabled
         */
		void set_rotate_speed(float rotate_speed, int acceleration=0, bool sync=false){
			this->control_serial->thread_lock();
			this->control_serial->generate_set_rotate_speed_command(this->address, rotate_speed, acceleration, sync);
			this->control_serial->send_command();
			this->control_serial->thread_unlock();
		}
	
	public: // Functions for generating and buffering commands
		/**
		 * @brief Generating and buffering the command of changing the address
		 * @param new_address The address you want to change to (0 ~ 255)
		 * @param save Whether save the change after rebooting or not
		 * @note If the new adress is invalid, the function will return
		 * @note If the long command reaches its max length, the command will not be appended
		 * @note [ ! ] If the command is not sent finally, the address buffered in the instance will still change, which may have some mistakes
		 * @note [ ! ] Try not to use
		 */
		void append_change_address_command(int new_address, bool save=true){
			if (new_address < 0 || new_address > 255){return;}
			this->control_serial->thread_lock();
			this->control_serial->generate_change_address_command(this->address, new_address, save);
			this->control_serial->append_command();
			this->control_serial->thread_unlock();
			this->address = new_address;
		}

		/**
         * @brief Generating and buffering the command of stopping the motor
         * @param sync Whether execute the command synchronously or not
		 * @note If the long command reaches its max length, the command will not be appended
         */
		void append_stop_command(bool sync=false){
			this->control_serial->thread_lock();
			this->control_serial->generate_stop_command(this->address, sync);
			this->control_serial->append_command();
			this->control_serial->thread_unlock();
		}

		/**
         * @brief Generating and buffering the command of setting rotation speed
         * @param rotate_speed Rotation speed in rpm (0 ~ 3000)
         * @param acceleration Acceleration grade of changing rotation speed (0 ~ 255)
         * @param sync Whether execute the command synchronously or not
         * @note +rotate_speed -> CW
         * @note "acceleration" is not real
		 * @note If the long command reaches its max length, the command will not be appended
         * @note [ ! ] "S_Vel_IS" in motor should be enabled
         */
		void append_set_rotate_speed_command(float rotate_speed, int acceleration=0, bool sync=false){
			this->control_serial->thread_lock();
			this->control_serial->generate_set_rotate_speed_command(this->address, rotate_speed, acceleration, sync);
			this->control_serial->append_command();
			this->control_serial->thread_unlock();
		}
};