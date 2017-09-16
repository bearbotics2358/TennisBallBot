/*
 * SwerveModule.cpp
 *
 *  Created on: Mar 21, 2016
 *      Author: hstechclub
 */

#include "SwerveModule.h"


const double MAX_RPM = /*4 * 40 * */ 4248 /* /60 / 10 */ ;
// 4 * encoder cpr * max rpm * 1 minute / 60s / 10 = maximum encoder value delta / .1s was used before we added in the encoder codesper rev config command
SwerveModule::SwerveModule(uint32_t turnMotorPort, uint32_t driveMotorPort)
: a_TurnMotor(turnMotorPort),
  a_DriveMotor(driveMotorPort)
{
	a_TurnMotor.SetControlMode(CANTalon::kPosition);
	a_TurnMotor.SetFeedbackDevice(TURN_ENCODER_TYPE);
	a_TurnMotor.SetSensorDirection(false);
	a_TurnMotor.SetP(0);
	a_TurnMotor.SetI(0);
	a_TurnMotor.SetD(0);
	a_TurnMotor.Set(0);

	a_DriveMotor.SetControlMode(CANTalon::kSpeed);
	a_DriveMotor.SetFeedbackDevice(CANTalon::QuadEncoder);
	a_DriveMotor.ConfigEncoderCodesPerRev(20);
	a_DriveMotor.SetSensorDirection(false);
	a_DriveMotor.SetP(0);
	a_DriveMotor.SetI(0);
	a_DriveMotor.SetD(0);
	a_DriveMotor.Set(0);

	distanceX = 0.0;
	distanceY = 0.0;
	lastPos = 0.0;
	scale = 0.031415926 * 2 * 1.2;

}

void SwerveModule::Update(float angle, float speed, float offset, float gyroValue)
{
	float currentPos = fabs(a_DriveMotor.GetEncPosition());
	float posDiff = currentPos - lastPos;
	// distance the wheel traveled since last cycle
	
	float phi = (gyroValue - GetAngle()) * M_PI / 180.0; 
	// this is an expression for the angle of a single wheel compared to the field's forward direction, in radians
	
	double dX = posDiff * sin(phi) * scale;
	double dY = posDiff * cos(phi) * scale;
	// here we pretend that the wheel only traveled in one direction in the time from last cycle to this one
	// and break out distance traveled into x and y components
	// which is not incorrect enough for us to not use, as 20 hz is OK for speeds as slow as we're travelling at
	
	distanceX += dX;
	distanceY += dY;

	/*
	if(fabs(angle - GetAngle()) > 180 && angle > 180) {
		angle -= 180;
		speed *= -1.0;
	} else if(fabs(angle - GetAngle()) > 180 && angle < 180) {
		angle += 180;
		speed *= -1.0;
	}
	*/
	
	// printf("The motor should be at speed: %f\n", speed);
	a_TurnMotor.Set((angle + offset) * ABSOLUTE_CONV_FACTOR);
	a_DriveMotor.Set(speed * MAX_RPM); 
	// native arg units are encoder delta / 0.1s, but we told the CANTalon our encodercodes per revolution and it uses RPM now
	lastPos = currentPos;

}

float SwerveModule::GetAngle()
{
	return a_TurnMotor.GetPosition() / ABSOLUTE_CONV_FACTOR;
}

float SwerveModule::GetSpeed()
{
	return a_DriveMotor.GetSpeed();
}

float SwerveModule::GetDistanceX()
{
	return distanceX;
}

float SwerveModule::GetDistanceY()
{
	return distanceY;
}

void SwerveModule::ResetDistanceX()
{
	distanceX = 0;
}

void SwerveModule::ResetDistanceY()
{
	distanceY = 0;
}

void SwerveModule::ResetDistances()
{
	ResetDistanceX();
	ResetDistanceY();
	a_DriveMotor.SetEncPosition(0);
}

void SwerveModule::InvertQuad()
{
	a_DriveMotor.SetSensorDirection(true);
}

void SwerveModule::InvertAnalog()
{
	a_TurnMotor.SetSensorDirection(true);
}

void SwerveModule::InvertDriveMotor()
{
	a_DriveMotor.SetInverted(true);
}

void SwerveModule::InvertTurnMotor()
{
	a_TurnMotor.SetInverted(true);
}

void SwerveModule::SetTurnPID(float turnP, float turnI, float turnD)
{
	a_TurnMotor.SetP(turnP);
	a_TurnMotor.SetI(turnI);
	a_TurnMotor.SetD(turnD);
}

void SwerveModule::SetDrivePIDF(float driveP, float driveI, float driveD, float driveF)
{
	a_DriveMotor.SetP(driveP);
	a_DriveMotor.SetI(driveI);
	a_DriveMotor.SetD(driveD);
	a_DriveMotor.SetF(driveF);
}

void SwerveModule::SetIzone(float izone)
{
	a_DriveMotor.SetIzone(izone);
	// just here as a reminder- this sets an area for I to accumulate outside of, so that it wouldn't accumulate if it's near target
}
