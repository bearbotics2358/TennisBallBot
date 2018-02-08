/*
 * Shooter.cpp
 *
 *  Created on: Sep 16, 2017
 *      Author: Alexis
 */

#include "Shooter.h"

const double MAX_RPM = 1000; // mod this in the future to a safe val; ensures consistent shooting

Shooter::Shooter(int firePort)
: a_ShooterTennis(firePort)
{
	a_ShooterTennis.Set(0);

}

void Shooter::Set(float speed)
{
	a_ShooterTennis.Set(speed * MAX_RPM); // ctre mag encoders and talon srx's work well together- no config needed for straightforward RPM control
}

float Shooter::GetSpeed()
{
	return 0;
	// return a_ShooterTennis.GetSpeed();
}

void Shooter::InvertQuad()
{
	// a_ShooterTennis.SetSensorDirection(true);
}

void Shooter::InvertFlyWheelMotor()
{
	a_ShooterTennis.SetInverted(true);
}

void Shooter::SetWheelPIDF(float wheelP, float wheelI, float wheelD, float wheelF)
{

}



