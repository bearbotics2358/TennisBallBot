/*
 * SwerveDrive.h
 *
 *  Created on: Mar 22, 2016
 *      Author: hstechclub
 */

#pragma once

#include <WPILib.h>

#include "Prefs.h"
#include "SwerveModule.h"

#include <math.h>
#include "SmartDashboard/SmartDashboard.h"
#include "SmartDashboard/SendableChooser.h"
#include "SmartDashboard/NamedSendable.h"
#include "SmartDashboard/SendableChooser.inc"
#include "SmartDashboard/SendableChooserBase.h"
#include "SmartDashboard/Sendable.h"
#include "Joystick.h"

class SwerveDrive { // Swerve drive with crab, forward and back and left and right only, twist, and true "swerve" modes
public:
	SwerveDrive(SwerveModule &FR, SwerveModule &FL, SwerveModule &BL, SwerveModule &BR, float Length, float Width);
	virtual ~SwerveDrive() = default;

	void Init();
	void InitSendableChooser();

	void Update(float XIn, float YIn, float ZIn, float gyroValue);
	void SetTwistingMode();
	void DisableTwist();
	void SetTwistingRelAngle(float gyroAngle, float angle);

	float GetDistanceY();
	float GetDistanceX();

	float GetDistanceRightX();
	float GetDistanceLeftX();
	float GetDistanceRightY();
	float GetDistanceLeftY();

	void Zero();

private:
	static const std::string CONTROL_TYPE_KEY;

	static const int         CONTROL_TYPE_SWERVE;
	static const std::string CONTROL_TYPE_SWERVE_KEY;
	static const int         CONTROL_TYPE_CRAB;
	static const std::string CONTROL_TYPE_CRAB_KEY;
	static const int         CONTROL_TYPE_SIMPLE_DRIVE;
	static const std::string CONTROL_TYPE_SIMPLE_DRIVE_KEY;
	static const int         CONTROL_TYPE_TURNING_DRIVE;
	static const std::string CONTROL_TYPE_TURNING_DRIVE_KEY;
	static const int         CONTROL_TYPE_TANK;
	static const std::string CONTROL_TYPE_TANK_KEY;
	static const int         CONTROL_TYPE_FL_TURN;
	static const std::string CONTROL_TYPE_FL_TURN_KEY;
	static const int         CONTROL_TYPE_FR_TURN;
	static const std::string CONTROL_TYPE_FR_TURN_KEY;

	SwerveModule &a_FrontRight; // "1"
	SwerveModule &a_FrontLeft; // "2"
	SwerveModule &a_BackLeft; // "3"
	SwerveModule &a_BackRight; // "4"

	float a_RobotLength;
	float a_RobotWidth;
	float a_ChassisRadius;

	frc::SendableChooser<std::string> a_ControlTypeChooser;
};
