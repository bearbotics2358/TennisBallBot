#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <memory>
#include <string>
#include <IterativeRobot.h>
#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>

#include "TennisBot.h"

TennisBot::TennisBot(void):
a_Joystick(JOYSTICK_PORT),
a_Joystick2(JOYSTICKTWO_PORT),

a_PDP(PDP_PORT),

a_FrontRight(FRONT_RIGHT_TURN, FRONT_RIGHT_MOVE),
a_FrontLeft(FRONT_LEFT_TURN, FRONT_LEFT_MOVE),
a_BackLeft(BACK_LEFT_TURN, BACK_LEFT_MOVE),
a_BackRight(BACK_RIGHT_TURN, BACK_RIGHT_MOVE),
a_Drive(a_FrontRight, a_FrontLeft, a_BackLeft, a_BackRight, CHASSIS_LENGTH, CHASSIS_WIDTH),
a_LRC(),
a_Accelerometer(I2C::kMXP,ADXL345_I2C::kRange_2G,0x53), // was 0x1D
a_Gyro(I2C::kMXP)

{
	rc = 0;
	tState = 0;
	SmartDashboard::init();
	a_Drive.Init();
	PREFS_FUNCTIONS // macro to invert drive
}

void TennisBot::RobotInit()
{
	a_Gyro.Cal();
}

void TennisBot::RobotPeriodic()
{
	a_Gyro.Update();
	// printf("Gyro Value: %f\n", a_Gyro.GetAngle());
	a_Accelerometer.GetAccelerations();

}

void TennisBot::DisabledInit()
{

}

void TennisBot::DisabledPeriodic()
{

	a_Drive.Update(0,0,0,0);
	SmartDashboard::PutNumber("gyro reg 0", a_Gyro.GetReg0());

	a_LRC.SetColor(0,0,100,0);
	a_LRC.SetColor(1,0,100,0);
	a_LRC.SetColor(2,0,100,0);
	SmartDashboard::PutNumber("Gyro, yum", a_Gyro.GetAngle());
	if(a_Joystick.GetRawButton(1)) {
		a_Gyro.Cal();
	}
	SmartDashboard::PutNumber("Front Right Speed", a_FrontRight.GetSpeed());
	SmartDashboard::PutNumber("Front Left Speed", a_FrontLeft.GetSpeed());
	SmartDashboard::PutNumber("Back Right Speed", a_BackRight.GetSpeed());
	SmartDashboard::PutNumber("Back Left Speed", a_BackLeft.GetSpeed());

	SmartDashboard::PutNumber("Front Right Angle", a_FrontRight.GetAngle());
	SmartDashboard::PutNumber("Front Left Angle", a_FrontLeft.GetAngle());
	SmartDashboard::PutNumber("Back Right Angle", a_BackRight.GetAngle());
	SmartDashboard::PutNumber("Back Left Angle", a_BackLeft.GetAngle());
}

void TennisBot::AutonomousInit()
{
	// a_Gyro.Cal();
	tState = 0;
	// a_Autonomous.Init();
}

void TennisBot::AutonomousPeriodic()
{
	SmartDashboard::PutNumber("Drive distance Y", a_Drive.GetDistanceY());
	SmartDashboard::PutNumber("Back Left", a_BackLeft.GetDistanceY());
	SmartDashboard::PutNumber("Back Right",a_BackRight.GetDistanceY());
	SmartDashboard::PutNumber("Front Left", a_FrontLeft.GetDistanceY());
	SmartDashboard::PutNumber("Front Right",  a_FrontRight.GetDistanceY());
}

void TennisBot::TeleopInit()
{

}

void TennisBot::TeleopPeriodic()
{
	SmartDashboard::PutNumber("Accelerometer X", a_Accelerometer.GetX());
	SmartDashboard::PutNumber("Accelerometer Y", a_Accelerometer.GetY());
	SmartDashboard::PutNumber("Accelerometer Z", a_Accelerometer.GetZ());

	a_LRC.SetColor(0,0,60,0);
	a_LRC.SetColor(1,0,60,0);
	a_LRC.SetColor(2,0,60,0);

	if(a_Joystick.GetRawButton(7)) {
		a_Drive.Zero();
	}

	if(a_Joystick2.GetRawButton(1) && a_Joystick2.GetRawAxis(2) >= .9) {
		a_Joystick2.SetRumble(GenericHID::RumbleType::kLeftRumble, 1);
		a_Joystick2.SetRumble(GenericHID::RumbleType::kRightRumble, 1);
	} else {
		a_Joystick2.SetRumble(GenericHID::RumbleType::kLeftRumble, 0);
		a_Joystick2.SetRumble(GenericHID::RumbleType::kRightRumble, 0);
	}

	if(a_Joystick.GetRawButton(1)) {
		a_Gyro.Cal();
	}

	SmartDashboard::PutNumber("Gyro, yum", a_Gyro.GetAngle());

	SmartDashboard::PutNumber("Drive distance X", a_Drive.GetDistanceX());
	SmartDashboard::PutNumber("Drive distance Y", a_Drive.GetDistanceY());

	SmartDashboard::PutNumber("Front Right Angle", a_FrontRight.GetAngle());
	SmartDashboard::PutNumber("Front Left Angle", a_FrontLeft.GetAngle());
	SmartDashboard::PutNumber("Back Right Angle", a_BackRight.GetAngle());
	SmartDashboard::PutNumber("Back Left Angle", a_BackLeft.GetAngle());

	SmartDashboard::PutNumber("Front Right Speed", a_FrontRight.GetSpeed());
	SmartDashboard::PutNumber("Front Left Speed", a_FrontLeft.GetSpeed());
	SmartDashboard::PutNumber("Back Right Speed", a_BackRight.GetSpeed());
	SmartDashboard::PutNumber("Back Left Speed", a_BackLeft.GetSpeed());

}


void TennisBot::TestInit()
{

}

void TennisBot::TestPeriodic()
{

}

START_ROBOT_CLASS(TennisBot);
