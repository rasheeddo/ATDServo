#include <stdlib.h>
#include <stdio.h>
#include <math.h>					
#include <chrono>
//using namespace std::chrono;
#include <unistd.h> 

#include "ATServoClass.h"

int main()
{

	ATServo servo;
	float currentDeg1;
	float currentDeg2;

	/*
	//// Example 1 ////
	servo.SpeedControl(1,360);		// SpeedControl(ID, target speed [deg/s])
	servo.SpeedControl(2,360);
	usleep(1000000);
	servo.MotorStop(1);				// Stop motor
	servo.MotorStop(2);
	usleep(1000000);
	servo.MotorRun(1);				// Continue run motor from latest command
	servo.MotorRun(2);
	//////////////////////////////////////////////////////////////////////////////////
	*/

	
	//// Example 2 ////
	servo.PositionControlMode1(1,160.4);		//PositionControlMode1(ID,target angle[deg]) run with full speed
	servo.PositionControlMode1(2,30.6);
	while(1)
	{
		currentDeg1 = servo.GetCurrentDeg(1);	//GetCurrentDeg(ID) return current angle in degree
		currentDeg2 = servo.GetCurrentDeg(2);
		printf("Current Deg1: %f\n", currentDeg1);
		printf("Current Deg2: %f\n", currentDeg2);
	}
	//////////////////////////////////////////////////////////////////////////////////
	

	/*
	//// Example 3 ////
	servo.PositionControlMode2(1,0,360);	//PositionControlMode2(ID, target angle[deg], target speed [deg/s])
	servo.PositionControlMode2(2,180,360);
	while(1)
	{
		currentDeg1 = servo.GetCurrentDeg(1);	//GetCurrentDeg(ID) return current angle in degree
		currentDeg2 = servo.GetCurrentDeg(2);
		printf("Current Deg1: %f\n", currentDeg1);
		printf("Current Deg2: %f\n", currentDeg2);
	}	
	//////////////////////////////////////////////////////////////////////////////////
	*/

	/*
	//// Example 4 ////
	servo.PositionControlMode3(1,45.5,1);		//PositionControlMode3(ID, target angle[deg], direction)  0: clockwire 1: counter-clockwise
	servo.PositionControlMode3(2,150.3,0);
	while(1)
	{
		currentDeg1 = servo.GetCurrentDeg(1);	//GetCurrentDeg(ID) return current angle in degree
		currentDeg2 = servo.GetCurrentDeg(2);
		printf("Current Deg1: %f\n", currentDeg1);
		printf("Current Deg2: %f\n", currentDeg2);
	}
	//////////////////////////////////////////////////////////////////////////////////
	*/

	/*
	//// Example 5 ////
	servo.PositionControlMode4(1,30.8,150,0);	//PositionControlMode4(ID, target angle[deg], target speed [deg/s], direction)  0: clockwire 1: counter-clockwise
	servo.PositionControlMode4(2,122.5,150,1);
	while(1)
	{
		currentDeg1 = servo.GetCurrentDeg(1);	//GetCurrentDeg(ID) return current angle in degree
		currentDeg2 = servo.GetCurrentDeg(2);
		printf("Current Deg1: %f\n", currentDeg1);
		printf("Current Deg2: %f\n", currentDeg2);
	}
	//////////////////////////////////////////////////////////////////////////////////
	*/

	//// Example 6 ////
	//servo.TorqueControl(1,100);		//TorqueControl(ID,raw torque value) the ratio of torque from -5000 to +5000
	//servo.TorqueControl(2,100);
	
	

	servo.portClose();

	return 0;
}
