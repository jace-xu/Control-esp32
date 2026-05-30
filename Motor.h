#pragma once
#include<ControlSerial.h>

/**
 * @brief This is the motor class
 * @note Direction: +rpm -> CW
 * @note == Verison 1.0.1 ==
 */
class Motor{
	private:
		ControlSerial* control_serial; /// Point to control serial object
		int address = 0; /// Address for this motor
	
	public:
		//Disabled copying and quotation
		Motor(const Motor&) = delete;
        Motor& operator=(const Motor&) = delete;
	
		/**
		 * @brief Create motor object
		 * @param serial The serial for controlling motors
		 * @param address Address for this motor
		 */
		Motor(ControlSerial& serial, int address){
			this->control_serial = &serial;
			if (address < 1){this->address = 1;}
			else if (address > 16){this->address = 16;}
			else{this->address = address;}
		}

	public:
		/**
		 * @brief Function for changing the address
		 * @param new_address The address you want to change to
		 * @note [ ! ] Try not to use
		 */
		void change_address(int new_address){
			if (new_address < 1 || new_address > 16){return;}
			this->control_serial->change_address(this->address, new_address);
			this->address = new_address;
		}

		/// @brief Stop this motor
		void stop(){this->control_serial->stop(this->address);}
		
		/**
		 * @brief Function for setting rotation speed
		 * @param rpm Rotation speed, must be less than 3000
		 * @note +rpm -> CW
		 * @note [ ! ] "S_Vel_IS" in motor should be enabled
		 */
		void set_rpm(float rpm){this->control_serial->set_rpm(this->address, rpm);}
};