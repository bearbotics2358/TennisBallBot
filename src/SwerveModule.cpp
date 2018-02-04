#include "SwerveModule.h"


const double MAX_RPM = /*4 * 40 * */ 4248 /* /60 / 10 */ ;
// 4 * encoder cpr * max rpm * 1 minute / 60s / 10 = maximum encoder value delta / .1s was used before we added in the encoder codesper rev config command
SwerveModule::SwerveModule(uint32_t turnMotorPort, uint32_t driveMotorPort)
: a_TurnMotor(turnMotorPort),
  a_DriveMotor(driveMotorPort)
{
	kSlotIdx = 0;
	kPIDLoopIdx = 0;
	kTimeoutMs = 10;

	a_TurnMotor.ConfigSelectedFeedbackSensor(TURN_ENCODER_TYPE, kPIDLoopIdx, kTimeoutMs);
	a_TurnMotor.SetSensorPhase(false);
	a_TurnMotor.Config_kP(kSlotIdx, 0, kTimeoutMs);
	a_TurnMotor.Config_kI(kSlotIdx, 0, kTimeoutMs);
	a_TurnMotor.Config_kD(kSlotIdx, 0, kTimeoutMs);
	a_TurnMotor.Set(0);

	a_DriveMotor.ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, kPIDLoopIdx, kTimeoutMs);
	// a_DriveMotor.ConfigEncoderCodesPerRev(20);
	a_DriveMotor.SetSensorPhase(false);
	a_DriveMotor.Config_kP(kSlotIdx, 0, kTimeoutMs);
	a_DriveMotor.Config_kI(kSlotIdx, 0, kTimeoutMs);
	a_DriveMotor.Config_kD(kSlotIdx, 0, kTimeoutMs);
	a_DriveMotor.Set(0);

	distanceX = 0.0;
	distanceY = 0.0;
	lastPos = 0.0;
	scale = 0.031415926 * 2 * 1.2;

}

void SwerveModule::Update(float angle, float speed, float offset, float gyroValue)
{
	float currentPos = fabs(a_DriveMotor.GetSelectedSensorPosition(0));
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
	a_TurnMotor.Set(ControlMode::Position, (angle + offset) * ABSOLUTE_CONV_FACTOR);
	a_DriveMotor.Set(ControlMode::Velocity, speed * MAX_RPM);
	// native arg units are encoder delta / 0.1s, but we told the CANTalon our encodercodes per revolution and it uses RPM now
	lastPos = currentPos;

}

float SwerveModule::GetAngle()
{
	return a_TurnMotor.GetSelectedSensorPosition(0) / ABSOLUTE_CONV_FACTOR;
}

float SwerveModule::GetSpeed()
{
	return a_DriveMotor.GetSensorCollection().GetPulseWidthVelocity();
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
	a_DriveMotor.GetSensorCollection().SetQuadraturePosition(0, kTimeoutMs);
}

void SwerveModule::InvertQuad()
{
	a_DriveMotor.SetSensorPhase(true);
}

void SwerveModule::InvertAnalog()
{
	a_TurnMotor.SetSensorPhase(true);
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
	a_TurnMotor.Config_kP(kSlotIdx, turnP, kTimeoutMs);
	a_TurnMotor.Config_kI(kSlotIdx, turnI, kTimeoutMs);
	a_TurnMotor.Config_kD(kSlotIdx, turnD, kTimeoutMs);
}

void SwerveModule::SetDrivePIDF(float driveP, float driveI, float driveD, float driveF)
{
	a_DriveMotor.Config_kP(kSlotIdx, driveP, kTimeoutMs);
	a_DriveMotor.Config_kI(kSlotIdx, driveI, kTimeoutMs);
	a_DriveMotor.Config_kD(kSlotIdx, driveD, kTimeoutMs);
	a_DriveMotor.Config_kF(kSlotIdx, driveF, kTimeoutMs);
}

void SwerveModule::SetIzone(float izone)
{
	a_DriveMotor.Config_IntegralZone(kSlotIdx, izone, kTimeoutMs);
	// just here as a reminder- this sets an area for I to accumulate outside of, so that it wouldn't accumulate if it's near target
}
