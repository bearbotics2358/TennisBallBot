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
	a_ShooterTennis.SetControlMode(CANTalon::kSpeed);
	a_ShooterTennis.SetFeedbackDevice(CANTalon::CtreMagEncoder_Relative);
	a_ShooterTennis.SetSensorDirection(true);
	a_ShooterTennis.SetP(SHOOTER_P);
	a_ShooterTennis.SetI(SHOOTER_I);
	a_ShooterTennis.SetD(SHOOTER_D);
	a_ShooterTennis.SetF(SHOOTER_F);
	a_ShooterTennis.SetIzone(SHOOTER_IZONE);
	a_ShooterTennis.Set(0);

}

void Shooter::Set(float speed)
{
	a_ShooterTennis.Set(speed * MAX_RPM); // ctre mag encoders and talon srx's work well together- no config needed for straightforward RPM control
}

float Shooter::GetSpeed()
{
	return a_ShooterTennis.GetSpeed();
}

void Shooter::InvertQuad()
{
	a_ShooterTennis.SetSensorDirection(true);
}

void Shooter::InvertFlyWheelMotor()
{
	a_ShooterTennis.SetInverted(true);
}

void Shooter::SetWheelPIDF(float wheelP, float wheelI, float wheelD, float wheelF)
{
	a_ShooterTennis.SetP(wheelP);
	a_ShooterTennis.SetI(wheelI);
	a_ShooterTennis.SetD(wheelD);
	a_ShooterTennis.SetF(wheelF);
}



