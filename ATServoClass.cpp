/*
	This is ATServo class from GYEMS servo class
	developed by Rasheed Kittinanthapanya, Attrac Lab.
	
*/

// C library headers
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

// Linux Headers
#include <fcntl.h>		// contains file control like O_RDWR
#include <errno.h>		// error integer and strerror() function
#include <termios.h>	// contains POSIX terminal control definitions
#include <unistd.h>     // write(), read(), close()
#include <linux/serial.h>
#include <sys/ioctl.h>

// Class
#include "ATServoClass.h"

ATServo::ATServo()
{
	RS485 = portInit();

};

int ATServo::portInit()
{
	// Open serial port
	int rs485 = open("/dev/ttyUSB0", O_RDWR);

	struct serial_rs485 rs485conf;
	// Create new termios struc
	struct termios tty;
	memset(&tty, 0, sizeof tty);

	// Read in existing settings, and handle any error
	if (rs485 < 0)
	{
		printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
	}

	/* Enable RS485 mode: */
	rs485conf.flags |= SER_RS485_ENABLED;
	rs485conf.delay_rts_before_send = 0.0;
	rs485conf.delay_rts_after_send = 0.0;
	rs485conf.flags |= SER_RS485_RX_DURING_TX;

	tty.c_cflag &= ~PARENB; 		// Clear parity bit, disabling parity (most common)
	tty.c_cflag &= ~CSTOPB;			// Clear stop field, only one stop bit used in communication (most common)
	tty.c_cflag |= CS8;				// 8 bits per byte (most common)
	tty.c_cflag &= ~CRTSCTS;		// Disable RTS/CTS hardware flow control (most common)
	tty.c_cflag |= CREAD | CLOCAL; 	// Turn on READ & ignore ctrl lines (CLOCAL = 1)

	tty.c_lflag &= ~ICANON;
	tty.c_lflag &= ~ECHO; 			// Disable echo
	tty.c_lflag &= ~ECHOE; 			// Disable erasure
	tty.c_lflag &= ~ECHONL; 		// Disable new-line echo
	tty.c_lflag &= ~ISIG; 			// Disable interpretation of INTR, QUIT and SUSP
	tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
	tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

	tty.c_oflag &= ~OPOST; 			// Prevent special interpretation of output bytes (e.g. newline chars)
	tty.c_oflag &= ~ONLCR; 			// Prevent conversion of newline to carriage return/line feed
	// tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT ON LINUX)
	// tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT ON LINUX)

	tty.c_cc[VTIME] = 0;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
	tty.c_cc[VMIN] = 0;

	// Set in/out baud rate to be 9600
	cfsetispeed(&tty, B115200);
	cfsetospeed(&tty, B115200);

	// Save tty settings, also checking for error
	if (tcsetattr(rs485, TCSANOW, &tty) != 0) 
	{
	    printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
	}

	return rs485;
};

void ATServo::portClose()
{
	close(RS485);

}

float ATServo::map(long value, long in_min, long in_max, long out_min, long out_max)
{
	return (((float)value - in_min) * (out_max - out_min) / (in_max-in_min) + out_min);
};

void ATServo::Int16ToByteData(unsigned int Data, unsigned char StoreByte[2])
{
  // unsigned int can store 16 bit int 
  StoreByte[0] = (Data & 0xFF00) >> 8;                  //High byte, most right of HEX
  StoreByte[1] = (Data & 0x00FF);                       //Low byte, most left of HEX
};

void ATServo::Int32ToByteData(unsigned long Data, unsigned char StoreByte[4])
{  
  // unsigned long can store 32 bit int 
  StoreByte[0] = (Data & 0xFF000000) >> 24;             //High byte, most right of HEX
  StoreByte[1] = (Data & 0x00FF0000) >> 16;
  StoreByte[2] = (Data & 0x0000FF00) >> 8;
  StoreByte[3] = (Data & 0x000000FF);                   //Low byte, most left of HEX
};

void ATServo::Int64ToByteData(unsigned long long Data, unsigned char StoreByte[8])
{  
  // unsigned long long can store 64 bit int 
  StoreByte[0] = (Data & 0xFF00000000000000) >> 56;       //High byte, most right of HEX
  StoreByte[1] = (Data & 0x00FF000000000000) >> 48;
  StoreByte[2] = (Data & 0x0000FF0000000000) >> 40;
  StoreByte[3] = (Data & 0x000000FF00000000) >> 32;     
  StoreByte[4] = (Data & 0x00000000FF000000) >> 24;     
  StoreByte[5] = (Data & 0x0000000000FF0000) >> 16;
  StoreByte[6] = (Data & 0x000000000000FF00) >> 8;
  StoreByte[7] = (Data & 0x00000000000000FF);             //Low byte, most left of HEX
};

unsigned int ATServo::Make12BitData(unsigned char loByte, unsigned char hiByte)
{
  unsigned int word;
  unsigned int Ang12Bit;

  word = (loByte & 0xFF) | ( (hiByte & 0xFF) << 8);       // construct 2 bytes data to a single 16 bits int
  Ang12Bit = map(word,0,16383,0,4095);                    // simply convert 16 bits to 12 bits data

  return Ang12Bit;
};

unsigned int ATServo::Make14BitData(unsigned char loByte, unsigned char hiByte)
{
  unsigned int Ang14Bit;

  Ang14Bit = (loByte & 0xFF) | ( (hiByte & 0xFF) << 8);       // construct 2 bytes data to a single 16 bits int
                                                              // even the total data length is 16 bits but the encoder can count only 14 bits

  return Ang14Bit;
};

float ATServo::GetCurrentDeg(int _ID)
{
	unsigned char ID = (unsigned char)_ID;
	unsigned char DataCheckByte = Header + EncoderCommand + ID + EncoderDataLen;
	float CurrentDeg;
	// send a command to servo to request a current position
	unsigned char packet[] = {Header, EncoderCommand, ID, ZeroDataLen, DataCheckByte};
	write(RS485, &packet, sizeof(packet));
	usleep(1700);   // A delay for next write command
	unsigned char read_buf[8];
	unsigned long EncoderData = 0;
	memset(&read_buf, '\0', sizeof(read_buf));
	int num_bytes = read(RS485, &read_buf, sizeof(read_buf));
	if (num_bytes<0)
	{
		printf("Error reading: %s \n", strerror(errno));
	}
	else if (num_bytes<8)
	{
		printf("Error reading: number of bytes are not 8bytes\n");
	}	
	else
	{
		//printf("Read %i bytes...\n", num_bytes);
		/*
		for(int i=0;i<num_bytes;i++)
		{
			printf("Byte %i : %x\n", i, read_buf[i]);
		}
		*/
		EncoderData = Make14BitData(read_buf[5],read_buf[6]);

	}

	CurrentDeg = (float)map(EncoderData,0,16383,0.0,360.0);

  return CurrentDeg;
}


void ATServo::MotorOff(int _ID)
{
	unsigned char ID = (unsigned char)_ID; 
	unsigned char DataCheckByte = Header + OffCommand + ID + ZeroDataLen;
	unsigned char packet[] = {Header, OffCommand, ID, ZeroDataLen, DataCheckByte};
	write(RS485, &packet, sizeof(packet));
	usleep(1700);   // A delay for next write command

};

void ATServo::MotorStop(int _ID)
{
	unsigned char ID = (unsigned char)_ID; 
	unsigned char DataCheckByte = Header + StopCommand + ID + ZeroDataLen;
	unsigned char packet[] = {Header, StopCommand, ID, ZeroDataLen, DataCheckByte};
	write(RS485, &packet, sizeof(packet));
	usleep(1700);   // A delay for next write command

}

void ATServo::MotorRun(int _ID)
{
	unsigned char ID = (unsigned char)_ID; 
	unsigned char  DataCheckByte = Header + RunCommand + ID + ZeroDataLen;
	unsigned char packet[] = {Header, RunCommand, ID, ZeroDataLen, DataCheckByte};
	write(RS485, &packet, sizeof(packet));
	usleep(1700);   // A delay for next write command
};

void ATServo::SetZero(int _ID)
{
	unsigned char ID = (unsigned char)_ID; 
	unsigned char DataCheckByte = Header + SetZeroCommand + ID + ZeroDataLen;
	unsigned char packet[] = {Header, SetZeroCommand, ID, ZeroDataLen, DataCheckByte};
	write(RS485, &packet, sizeof(packet));
	usleep(1700);   // A delay for next write command
};

void ATServo::TorqueControl(int _ID, unsigned int Torque)
{
  // Torque is a raw value, actual torque depends on the motor spec
	unsigned char ID = (unsigned char)_ID; 
	unsigned char FrameCheckSum = Header + TorqueCommand + TorqueDataLen + ID;
	unsigned char TorqueByte[2];
	Int16ToByteData(Torque,TorqueByte);
	unsigned char DataCheckByte = TorqueByte[1] + TorqueByte[0];

	unsigned char packet[] = {Header, TorqueCommand, ID, TorqueDataLen, FrameCheckSum, TorqueByte[1], TorqueByte[0], DataCheckByte};
	write(RS485, &packet, sizeof(packet));
	usleep(1700);   // A delay for next write command

};

void ATServo::SpeedControl(int _ID, float DPS)
{
	unsigned char ID = (unsigned char)_ID; 
	// DPS is degree per second
	float SpeedLSB = DPS*100;
	unsigned char FrameCheckSum = Header + SpeedCommand + SpeedDataLen + ID;
	unsigned char SpeedByte[4];
	Int32ToByteData((int)SpeedLSB,SpeedByte);
	unsigned char DataCheckByte = SpeedByte[3] + SpeedByte[2] + SpeedByte[1] + SpeedByte[0];

	unsigned char packet[] = {Header, SpeedCommand, ID, SpeedDataLen, FrameCheckSum, SpeedByte[3], SpeedByte[2], SpeedByte[1], SpeedByte[0], DataCheckByte};
	write(RS485, &packet, sizeof(packet));
	usleep(1700);   // A delay for next write command

};

void ATServo::PositionControlMode1(int _ID, float Deg)
{ 
	unsigned char ID = _ID;
	unsigned long long DegLSB = (unsigned long long)(Deg*100);
	unsigned char FrameCheckSum = Header + Pos1Command + ID + Pos1DataLen;
	unsigned char PositionByte[8];
	Int64ToByteData(DegLSB,PositionByte);
	unsigned char DataCheckByte = PositionByte[7] + PositionByte[6] + PositionByte[5] + PositionByte[4] + PositionByte[3] + PositionByte[2] + PositionByte[1] + PositionByte[0];
	unsigned char packet[] = {Header, Pos1Command, ID, Pos1DataLen, FrameCheckSum, PositionByte[7], 
								PositionByte[6], PositionByte[5], PositionByte[4], PositionByte[3], 
								PositionByte[2], PositionByte[1], PositionByte[0], DataCheckByte};
	write(RS485, &packet, sizeof(packet));
	usleep(2000);   // A delay for next write command
}

void ATServo::PositionControlMode2(int _ID, float Deg, unsigned long DPS)
{
	unsigned char ID = _ID;
	unsigned long long DegLSB = (unsigned long long)(Deg*100);
	unsigned long SpeedLSB = (unsigned long)DPS*100;
	unsigned char FrameCheckSum = Header + Pos2Command + ID + Pos2DataLen;
	unsigned char PositionByte[8];
	unsigned char SpeedByte[4];
	Int32ToByteData(SpeedLSB,SpeedByte);
	Int64ToByteData(DegLSB,PositionByte);
	unsigned char DataCheckByte = PositionByte[7] + PositionByte[6] + PositionByte[5] + PositionByte[4] + PositionByte[3] + PositionByte[2] + PositionByte[1] + PositionByte[0] +  SpeedByte[3] + SpeedByte[2] + SpeedByte[1] + SpeedByte[0];
	unsigned char packet[] = {Header, Pos2Command, ID, Pos2DataLen, FrameCheckSum, PositionByte[7], 
							PositionByte[6], PositionByte[5], PositionByte[4], PositionByte[3], PositionByte[2], PositionByte[1], 
							PositionByte[0], SpeedByte[3], SpeedByte[2], SpeedByte[1], SpeedByte[0], DataCheckByte};
	write(RS485, &packet, sizeof(packet));
	usleep(2300);   // A delay for next write command
}

void ATServo::PositionControlMode3(int _ID, float Deg, unsigned char Direction)
{
	unsigned char ID = _ID;
	//printf("input deg %f\n", Deg);
	unsigned long DegLSB = (unsigned long)(Deg*100);
	unsigned char FrameCheckSum = Header + Pos3Command + ID + Pos3DataLen;
	unsigned char PositionByte[4];
	Int32ToByteData(DegLSB,PositionByte);
	unsigned char DataCheckByte = Direction + PositionByte[3] + PositionByte[2] + PositionByte[1];
	unsigned char packet[] = {Header, Pos3Command, ID, Pos3DataLen, FrameCheckSum, Direction, 
							PositionByte[3], PositionByte[2], PositionByte[1], DataCheckByte};
	write(RS485, &packet, sizeof(packet));
	usleep(2300);   // A delay for next write command
}

void ATServo::PositionControlMode4(int _ID, float Deg, float DPS, unsigned char Direction)
{
	unsigned char ID = _ID;
	unsigned long DegLSB = (unsigned long)(Deg*100);
	unsigned long SpeedLSB = (unsigned long)(DPS*100);
	unsigned char FrameCheckSum = Header + Pos4Command + ID + Pos4DataLen ;
	unsigned char PositionByte[4];
	unsigned char SpeedByte[4];
	Int32ToByteData(DegLSB,PositionByte);
	Int32ToByteData(SpeedLSB,SpeedByte);
	unsigned char DataCheckByte = Direction + PositionByte[3] + PositionByte[2] + PositionByte[1] + SpeedByte[3] + SpeedByte[2] + SpeedByte[1] + SpeedByte[0];
	unsigned char packet[] = {Header, Pos4Command, ID, Pos4DataLen, FrameCheckSum, Direction, PositionByte[3], 
							PositionByte[2], PositionByte[1], SpeedByte[3], SpeedByte[2], SpeedByte[1], SpeedByte[0], DataCheckByte};
	write(RS485, &packet, sizeof(packet));
	usleep(2500);   // A delay for next write command

}










