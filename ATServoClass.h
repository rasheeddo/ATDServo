/*
	This is ATServo class from GYEMS servo class
	developed by Rasheed Kittinanthapanya, Attrac Lab.
	
*/

#ifndef ATSERVOCLASS_H
#define ATSERVOCLASS_H

class ATServo
{

	private:

		int RS485;

		// Header Byte //
		unsigned char Header = 0x3E;

		// Command Byte //
		unsigned char EncoderCommand = 0x90;
		unsigned char OffCommand = 0x80;
		unsigned char StopCommand = 0x81;
		unsigned char RunCommand = 0x88;
		unsigned char SetZeroCommand = 0x19;
		unsigned char TorqueCommand = 0xA1;
		unsigned char SpeedCommand = 0xA2;
		unsigned char Pos1Command = 0xA3;
		unsigned char Pos2Command = 0xA4;
		unsigned char Pos3Command = 0xA5;
		unsigned char Pos4Command = 0xA6;

		// Data Length Byte //
		unsigned char ZeroDataLen = 0x00;
		unsigned char TorqueDataLen = 0x02;
		unsigned char EncoderDataLen = 0x00;
		unsigned char SpeedDataLen = 0x04;
		unsigned char Pos1DataLen = 0x08;
		unsigned char Pos2DataLen = 0x0C;
		unsigned char Pos3DataLen = 0x04;
		unsigned char Pos4DataLen = 0x08;

	public:

		ATServo();

		////////////////////////////////////////////////////////////////////////////////////////////////////
		///////////////////////////////////////// Initialization ///////////////////////////////////////////
		////////////////////////////////////////////////////////////////////////////////////////////////////

		int portInit();
		// portInit : initialize and config serial port to use as RS485 mode
		// return a port which use for write and read

		void portClose();

		////////////////////////////////////////////////////////////////////////////////////////////////////
		///////////////////////////////////// Data Converting function /////////////////////////////////////
		////////////////////////////////////////////////////////////////////////////////////////////////////

		float map(long value, long in_min, long in_max, long out_min, long out_max);

		void Int16ToByteData(unsigned int Data, unsigned char StoreByte[2]);
		// Int16ToByteData: split an unsigned int into 2 bytes data
		// unsigned int stores 16 bits as an integer
		// Pass by reference using array StoreByte[2], this is output of the function

		void Int32ToByteData(unsigned long Data, unsigned char StoreByte[4]);
		// Int32ToByteData: split an unsigned long into 4 bytes data
		// unsigned long stores 32 bits as an integer
		// Pass by reference using array StoreByte[4], this is output of the function

		void Int64ToByteData(unsigned long long Data, unsigned char StoreByte[8]);
		// Int64ToByteData: split an unsigned long long into 8 bytes data
		// unsigned long long stores 64 bits as an integer
		// Pass by reference using array StoreByte[8], this is output of the function

		unsigned int Make12BitData(unsigned char loByte, unsigned char hiByte);
		// Mak12BitData: construct 2 bytes data from low byte and high byte into a single WORD (16 bits)
		// then convert that 16 bits data to 12 bits data  (for encoder data of RMD-S series)

		unsigned int Make14BitData(unsigned char loByte, unsigned char hiByte);
		// Mak14BitData: construct 2 bytes data from low byte and high byte into a single WORD (16 bits)
		// then convert that 16 bits data to 14 bits data  (for encoder data of RMD-L series)


		////////////////////////////////////////////////////////////////////////////////////////////////////
		///////////////////////////////////// Servo control function ///////////////////////////////////////
		////////////////////////////////////////////////////////////////////////////////////////////////////

		float GetCurrentDeg(int _ID);
		// GetCurrentDEG: a command to send data to the servo to request the current position
		// after a servo receive the command, it will reply back as a encoder data as 12bit (0-4095)
		// return an angle data in degree

		void MotorOff(int _ID);
		// MotorOff: Turn off the motor and clear the motor running status and the previously received control commands.

		void MotorStop(int _ID);
		// MotorStop: stop the motor but does not clear the motor running status and the previously received control command.

		void MotorRun(int _ID);
		// MotorStop: continue run the motor from MotorStop(), control mode would be same as before stop

		void SetZero(int _ID);
		// SetZero: set current motor position as zero position
		// NOTE from factory instruction manual
		// 1． The command will be valid after power on again.
		// 2． The command will write the zero point to the drive FLASH memory. Multiple writes will affect the chip life. 
		// It is not recommended to use it frequently

		void TorqueControl(int _ID, unsigned int Torque);
		// TorqueControl: closed loop torque control
		// Input is the ratio of torque from -5000 to +5000, the actual torque depends on motor's model

		void SpeedControl(int _ID, float DPS);
		// SpeedControl: this function is to control the speed of servo in degree per second (dps or DPS). max. value is 720DPS.
		// The direction of the motor is determined from the sign of input DPS

		void PositionControlMode1(int _ID, float Deg);
		// PositionControlMode1: The servo will run with maximum speed as config (720dps as default) to the desired position "Deg"
		// This is a multi turn mode, the input Deg can be from -3600deg to 3600deg (10rounds)

		void PositionControlMode2(int _ID, float Deg, unsigned long DPS);
		// PositionControlMode2: The servo will run with the specified speed as "DPS" to the desired angle "Deg"
		// This is a multi turn mode, the input Deg can be from -3600deg to 3600deg (10rounds), and max. speed is 720dps.

		void PositionControlMode3(int _ID, float Deg, unsigned char Direction);
		// PositionControlMode3: The servo will run with maximum speed as config (720dps as default) to the desired position "Deg" 
		// with the desired direction as "Direction", 0x00 for clockwise and 0x01 with counter clockwise

		void PositionControlMode4(int _ID, float Deg, float DPS, unsigned char Direction);
		// PositionControlMode4: The servo will run with the specified speed as "DPS" to the desired angle "Deg" 
		// with desired direction as "Direction", 0x00 for clockwise and 0x01 with counter clockwise

};



#endif