/*
 * SwerveDrive.cpp
 *
 *  Created on: Mar 22, 2016
 *      Author: hstechclub
 */

#include "SwerveDrive.h"
#include "Joystick.h"
#include "SmartDashboard/SmartDashboard.h"
#include "SmartDashboard/SendableChooser.h"
#include "SmartDashboard/NamedSendable.h"
#include "SmartDashboard/SendableChooser.inc"
#include "SmartDashboard/SendableChooserBase.h"
#include "SmartDashboard/Sendable.h"

const std::string SwerveDrive::CONTROL_TYPE_KEY = "Drive Scheme";

const int         SwerveDrive::CONTROL_TYPE_SWERVE = 0;
const std::string SwerveDrive::CONTROL_TYPE_SWERVE_KEY = "Swerve Drive";
const int         SwerveDrive::CONTROL_TYPE_CRAB   = 1;
const std::string SwerveDrive::CONTROL_TYPE_CRAB_KEY   = "Crab Drive";
const int         SwerveDrive::CONTROL_TYPE_SIMPLE_DRIVE = 2;
const std::string SwerveDrive::CONTROL_TYPE_SIMPLE_DRIVE_KEY = "Simple Drive";
const int         SwerveDrive::CONTROL_TYPE_TURNING_DRIVE = 3;
const std::string SwerveDrive::CONTROL_TYPE_TURNING_DRIVE_KEY = "Turning Drive";
const int         SwerveDrive::CONTROL_TYPE_TANK = 4;
const std::string SwerveDrive::CONTROL_TYPE_TANK_KEY = "Tank Drive";
const int         SwerveDrive::CONTROL_TYPE_FL_TURN = 5;
const std::string SwerveDrive::CONTROL_TYPE_FL_TURN_KEY = "FL Turn Drive";
const int         SwerveDrive::CONTROL_TYPE_FR_TURN = 6;
const std::string SwerveDrive::CONTROL_TYPE_FR_TURN_KEY = "FR Turn Drive";

bool isTwisting = false;
float turnAngle = 0;

SwerveDrive::SwerveDrive(SwerveModule &FR, SwerveModule &FL, SwerveModule &BL, SwerveModule &BR, float Length, float Width)
: a_FrontRight(FR),
  a_FrontLeft(FL),
  a_BackLeft(BL),
  a_BackRight(BR)
{
	a_RobotLength = Length;
	a_RobotWidth = Width;
	a_ChassisRadius = sqrt(pow(a_RobotLength, 2) + pow(a_RobotWidth, 2));
	a_ControlTypeChooser.AddDefault(CONTROL_TYPE_SWERVE_KEY, CONTROL_TYPE_SWERVE_KEY);
	a_ControlTypeChooser.AddObject(CONTROL_TYPE_CRAB_KEY, CONTROL_TYPE_CRAB_KEY);
	a_ControlTypeChooser.AddObject(CONTROL_TYPE_SIMPLE_DRIVE_KEY, CONTROL_TYPE_SIMPLE_DRIVE_KEY);
	a_ControlTypeChooser.AddObject(CONTROL_TYPE_TANK_KEY, CONTROL_TYPE_TANK_KEY);
	a_ControlTypeChooser.AddObject(CONTROL_TYPE_FL_TURN_KEY, CONTROL_TYPE_FL_TURN_KEY);
	a_ControlTypeChooser.AddObject(CONTROL_TYPE_FR_TURN_KEY, CONTROL_TYPE_FR_TURN_KEY);
};

void SwerveDrive::Init()
{
	SmartDashboard::PutData(CONTROL_TYPE_KEY, &a_ControlTypeChooser); // this is important to call first, otherwise drive schemes may not initialize properly
	a_FrontRight.SetDrivePIDF(FRONT_RIGHT_DRIVE_PIDF);
	a_FrontRight.SetTurnPID(FRONT_RIGHT_TURN_PID);
	a_FrontLeft.SetDrivePIDF(FRONT_LEFT_DRIVE_PIDF);
	a_FrontLeft.SetTurnPID(FRONT_LEFT_TURN_PID);
	a_BackRight.SetDrivePIDF(BACK_RIGHT_DRIVE_PIDF);
	a_BackRight.SetTurnPID(BACK_RIGHT_TURN_PID);
	a_BackRight.SetIzone(BACK_RIGHT_IZONE);
	a_BackLeft.SetDrivePIDF(BACK_LEFT_DRIVE_PIDF);
	a_BackLeft.SetTurnPID(BACK_LEFT_TURN_PID);

	a_FrontRight.Update(0,0,0,0);
	a_FrontLeft.Update(0,0,0,0);
	a_BackRight.Update(0,0,0,0);
	a_BackLeft.Update(0,0,0,0);
}

void SwerveDrive::InitSendableChooser()
{
	SmartDashboard::PutData(CONTROL_TYPE_KEY, &a_ControlTypeChooser);
}

void SwerveDrive::Update(float XIn, float YIn, float ZIn, float gyroValue)
{
	// float kJoystickDeadzone = 0.1;
	// float range = 1 - kJoystickDeadzone;
	float zInput = ZIn; // Rotation Clockwise
	float xInput = -1.0 * XIn; // Strafe
	float yInput = YIn; // Forward

	//	if(fabs(zInput) < kJoystickDeadzone + 0.1) {
	//		zInput = 0;
	//	} else if(zInput > 0){
	//		zInput = (zInput - (kJoystickDeadzone + 0.1)) / range;
	//	} else {
	//		zInput = (zInput + (kJoystickDeadzone + 0.1)) / range;
	//	}
	//
	//	if(fabs(xInput) < kJoystickDeadzone) {
	//		xInput = 0;
	//	} else if(xInput > 0){
	//		xInput = (xInput - kJoystickDeadzone) / range;
	//	} else {
	//		xInput = (xInput + kJoystickDeadzone) / range;
	//	}
	//
	//	if(fabs(yInput) < kJoystickDeadzone) {
	//		yInput = 0;
	//	} else if(yInput > 0){
	//		yInput = (yInput - kJoystickDeadzone) / range;
	//	} else {
	//		yInput = (yInput + kJoystickDeadzone) / range;
	//	}

	float temp = yInput * cos(gyroValue * M_PI / 180) + xInput * sin(gyroValue * M_PI / 180); // This block of commands makes this thing field oriented
	xInput = -yInput * sin(gyroValue * M_PI / 180) + xInput * cos(gyroValue * M_PI / 180);
	yInput = temp;

	float diff = fabs(gyroValue - turnAngle);

	std::string controlType;
	SmartDashboard::PutData(CONTROL_TYPE_KEY, &a_ControlTypeChooser);
	controlType = a_ControlTypeChooser.GetSelected();

	if (controlType == "") {
		std::cout << "error reading control type" << std::endl;
		return;
	}
	if(isTwisting) {
		controlType = CONTROL_TYPE_TURNING_DRIVE_KEY;
	}

	float A = xInput - zInput * (a_RobotLength / a_ChassisRadius);
	float B = xInput + zInput * (a_RobotLength / a_ChassisRadius);
	float C = yInput - zInput * (a_RobotWidth / a_ChassisRadius);
	float D = yInput + zInput * (a_RobotWidth / a_ChassisRadius);

	float max = -9 * pow(10,4);

	double frSpeed = 0.0;
	double frAngle = 0.0;

	double flSpeed = 0.0;
	double flAngle = 0.0;

	double blSpeed = 0.0;
	double blAngle = 0.0;

	double brSpeed = 0.0;
	double brAngle = 0.0;

	if(controlType == CONTROL_TYPE_SWERVE_KEY) {
		frSpeed = sqrt(pow(B,2) + pow(C,2));
		flSpeed = sqrt(pow(B,2) + pow(D,2));
		blSpeed = sqrt(pow(A,2) + pow(D,2));
		brSpeed = sqrt(pow(A,2) + pow(C,2));

		max = frSpeed;
		if(flSpeed > max) {
			max = flSpeed;
		}
		if(blSpeed > max) {
			max = blSpeed;
		}
		if(brSpeed > max) {
			max = brSpeed;
		}
		if(max > 1) { // This is done so that if a speed greater than 1 is calculated, all are reduced proportionally
			frSpeed /= max;
			flSpeed /= max;
			blSpeed /= max;
			brSpeed /= max;
		}

		//  atan2 outputs values in a manner similar to what is shown on the below diagram

		////////////////////
		//        90      //
		//        //      //
		//		  //      //
		//		  //      //
		//180 or - 180///0//
		//        //      //
		//        //      //
		//        //      //
		//       -90      //
		////////////////////

		frAngle = (atan2(C,B) * 180.0 / M_PI);
		flAngle = (atan2(D,B) * 180.0 / M_PI);
		blAngle = (atan2(D,A) * 180.0 / M_PI);
		brAngle = (atan2(C,A) * 180.0 / M_PI);


		// Subtract 90 from all

		////////////////////
		//        0	      //
		//        //      //
		//		  //      //
		//		  //      //
		//90 or - 270//-90//
		//        //      //
		//        //      //
		//        //      //
		//  	 -180     //
		////////////////////

		frAngle -= 90;
		flAngle -= 90;
		brAngle -= 90;
		blAngle -= 90;

		/* the following code is commented out because on the swerve encoders we have now clockwise is negative. uncomment if the opposite is true.
				// Multiply all by -1

				        ////////////////////
					    //        0	      //
						//        //      //
						//		  //      //
						//		  //      //
						//-90 or 270///90///
						//        //      //
						//        //      //
						//        //      //
						//  	 180     //
						////////////////////

				frAngle *= -1.0;
				flAngle *= -1.0;
				brAngle *= -1.0;
				blAngle *= -1.0;
				
			// removed the part where the "wrapping" occurs is adjusted to the forward direction- 
			// forward is the most common direction to drive in, so having the wrap anywhere near there is not a great idea
		 */

	} else if(controlType == CONTROL_TYPE_FL_TURN_KEY) {
		double flX = xInput;
		double flY = yInput;
		double frX = xInput;
		double frY = yInput - zInput;
		double blX = xInput - zInput;
		double blY = yInput;
		double brX = xInput - zInput * (a_RobotLength / a_ChassisRadius);
		double brY = yInput - zInput * (a_RobotWidth / a_ChassisRadius);

		frSpeed = sqrt(pow(frX,2) + pow(frY,2));
		flSpeed = sqrt(pow(flX,2) + pow(flY,2));
		blSpeed = sqrt(pow(blX,2) + pow(blY,2));
		brSpeed = sqrt(pow(brX,2) + pow(brY,2));

		max = frSpeed;
		if(flSpeed > max) {
			max = flSpeed;
		}
		if(blSpeed > max) {
			max = blSpeed;
		}
		if(brSpeed > max) {
			max = brSpeed;
		}
		if(max > 1) { // This is done so that if a speed greater than 1 is calculated, all are reduced proportionally
			frSpeed /= max;
			flSpeed /= max;
			blSpeed /= max;
			brSpeed /= max;
		}

		//  atan2 outputs values in a manner similar to what is shown on the below diagram

		////////////////////
		//        90      //
		//        //      //
		//		  //      //
		//		  //      //
		//180 or - 180///0//
		//        //      //
		//        //      //
		//        //      //
		//       -90      //
		////////////////////

		frAngle = (atan2(frX,frY) * 180.0 / M_PI);
		flAngle = (atan2(flX,flY) * 180.0 / M_PI);
		blAngle = (atan2(blX,blY) * 180.0 / M_PI);
		brAngle = (atan2(brX,brY) * 180.0 / M_PI);


		// Subtract 90 from all

		////////////////////
		//        0	      //
		//        //      //
		//		  //      //
		//		  //      //
		//90 or - 270//-90//
		//        //      //
		//        //      //
		//        //      //
		//  	 -180     //
		////////////////////

		frAngle -= 90;
		flAngle -= 90;
		brAngle -= 90;
		blAngle -= 90;
	} else if(controlType == CONTROL_TYPE_SIMPLE_DRIVE_KEY) {
		if(fabs(xInput) > fabs(yInput)) {
			float setAngle = -90;
			frAngle = setAngle;
			flAngle = setAngle;
			blAngle = setAngle;
			brAngle = setAngle;

			float setSpeed = xInput;
			frSpeed = setSpeed;
			flSpeed = setSpeed;
			blSpeed = setSpeed;
			brSpeed = setSpeed;
		} else {
			float setAngle = 0;
			frAngle = setAngle;
			flAngle = setAngle;
			blAngle = setAngle;
			brAngle = setAngle;

			float setSpeed = yInput;
			frSpeed = setSpeed;
			flSpeed = setSpeed;
			blSpeed = setSpeed;
			brSpeed = setSpeed;
		}
	} else if(controlType == CONTROL_TYPE_TURNING_DRIVE_KEY) {
		flAngle = 0;
		frAngle = 0;
		blAngle = 0;
		brAngle = 0;

		if(gyroValue < turnAngle - 3) {
			if(diff > 10) {
				frSpeed = -0.3;
				flSpeed = 0.3;
				blSpeed = 0.3;
				brSpeed = -0.3;

			} else {
				frSpeed = -0.2;
				flSpeed = 0.2;
				blSpeed = 0.2;
				brSpeed = -0.2;
			}
		} else if(gyroValue > turnAngle + 3) {
			if(diff > 10) {
				frSpeed = 0.3;
				flSpeed = -0.3;
				blSpeed = -0.3;
				brSpeed = 0.3;
			} else {
				frSpeed = 0.2;
				flSpeed = -0.2;
				blSpeed = -0.2;
				brSpeed = 0.2;
			}
		} else {
			frSpeed = 0;
			flSpeed = 0;
			blSpeed = 0;
			brSpeed = 0;
			DisableTwist();
		}
	} else if(controlType == CONTROL_TYPE_TANK_KEY) {
		if(fabs(xInput) > 0.125) {
			flAngle = -90.0;
			frAngle = -90.0;
			blAngle = -90.0;
			brAngle = -90.0;

			frSpeed = xInput;
			flSpeed = xInput;
			blSpeed = xInput;
			brSpeed = xInput;
		} else {
			flAngle = 0;
			frAngle = 0;
			blAngle = 0;
			brAngle = 0;

			frSpeed = yInput - zInput;
			flSpeed = yInput + zInput;
			blSpeed = yInput + zInput;
			brSpeed = yInput - zInput;

			max = fabs(frSpeed);
			if(fabs(flSpeed) > max) {
				max = fabs(flSpeed);
			}
			if(fabs(blSpeed) > max) {
				max = fabs(blSpeed);
			}
			if(fabs(brSpeed) > max) {
				max = fabs(brSpeed);
			}
			if(max > 1) { // This is done so that if a speed greater than 1 is calculated, all are reduced proportionally
				frSpeed /= max;
				flSpeed /= max;
				blSpeed /= max;
				brSpeed /= max;
			}
		}
	} else if(controlType == CONTROL_TYPE_FR_TURN_KEY) {
		double flX = xInput;
		double flY = yInput + zInput;
		double frX = xInput;
		double frY = yInput;
		double blX = xInput - zInput * (a_RobotLength / a_ChassisRadius);
		double blY = yInput + zInput * (a_RobotLength / a_ChassisRadius);
		double brX = xInput - zInput;
		double brY = yInput;

		frSpeed = sqrt(pow(frX,2) + pow(frY,2));
		flSpeed = sqrt(pow(flX,2) + pow(flY,2));
		blSpeed = sqrt(pow(blX,2) + pow(blY,2));
		brSpeed = sqrt(pow(brX,2) + pow(brY,2));

		max = frSpeed;
		if(flSpeed > max) {
			max = flSpeed;
		}
		if(blSpeed > max) {
			max = blSpeed;
		}
		if(brSpeed > max) {
			max = brSpeed;
		}
		if(max > 1) { // This is done so that if a speed greater than 1 is calculated, all are reduced proportionally
			frSpeed /= max;
			flSpeed /= max;
			blSpeed /= max;
			brSpeed /= max;
		}

		//  atan2 outputs values in a manner similar to what is shown on the below diagram

		////////////////////
		//        90      //
		//        //      //
		//		  //      //
		//		  //      //
		//180 or - 180///0//
		//        //      //
		//        //      //
		//        //      //
		//       -90      //
		////////////////////

		frAngle = (atan2(frX,frY) * 180.0 / M_PI);
		flAngle = (atan2(flX,flY) * 180.0 / M_PI);
		blAngle = (atan2(blX,blY) * 180.0 / M_PI);
		brAngle = (atan2(brX,brY) * 180.0 / M_PI);


		// Subtract 90 from all

		////////////////////
		//        0	      //
		//        //      //
		//		  //      //
		//		  //      //
		//90 or - 270//-90//
		//        //      //
		//        //      //
		//        //      //
		//  	 -180     //
		////////////////////

		frAngle -= 90;
		flAngle -= 90;
		brAngle -= 90;
		blAngle -= 90;
	} else if(controlType == CONTROL_TYPE_SIMPLE_DRIVE_KEY) {
		if(fabs(xInput) > fabs(yInput)) {
			float setAngle = -90;
			frAngle = setAngle;
			flAngle = setAngle;
			blAngle = setAngle;
			brAngle = setAngle;

			float setSpeed = xInput;
			frSpeed = setSpeed;
			flSpeed = setSpeed;
			blSpeed = setSpeed;
			brSpeed = setSpeed;
		} else {
			float setAngle = 0;
			frAngle = setAngle;
			flAngle = setAngle;
			blAngle = setAngle;
			brAngle = setAngle;

			float setSpeed = yInput;
			frSpeed = setSpeed;
			flSpeed = setSpeed;
			blSpeed = setSpeed;
			brSpeed = setSpeed;
		}
	} else if(controlType == CONTROL_TYPE_TURNING_DRIVE_KEY) {
		flAngle = 0;
		frAngle = 0;
		blAngle = 0;
		brAngle = 0;

		if(gyroValue < turnAngle - 3) {
			if(diff > 10) {
				frSpeed = -0.3;
				flSpeed = 0.3;
				blSpeed = 0.3;
				brSpeed = -0.3;

			} else {
				frSpeed = -0.2;
				flSpeed = 0.2;
				blSpeed = 0.2;
				brSpeed = -0.2;
			}
		} else if(gyroValue > turnAngle + 3) {
			if(diff > 10) {
				frSpeed = 0.3;
				flSpeed = -0.3;
				blSpeed = -0.3;
				brSpeed = 0.3;
			} else {
				frSpeed = 0.2;
				flSpeed = -0.2;
				blSpeed = -0.2;
				brSpeed = 0.2;
			}
		} else {
			frSpeed = 0;
			flSpeed = 0;
			blSpeed = 0;
			brSpeed = 0;
			DisableTwist();
		}
	} else {
		float setAngle = atan2(yInput, xInput) * 180 / M_PI;
		setAngle -= 90;

		frAngle = setAngle;
		flAngle = setAngle;
		blAngle = setAngle;
		brAngle = setAngle;

		float setSpeed = sqrt(pow(xInput,2) + pow(yInput,2)); // find the r of the joystick vector, if you think about it in polar coordinates

		frSpeed = setSpeed;
		flSpeed = setSpeed;
		blSpeed = setSpeed;
		brSpeed = setSpeed;
	}

	SmartDashboard::PutNumber("Front Right Theoretical Speed", frSpeed * 4248);
	SmartDashboard::PutNumber("Front Left Theoretical Speed", flSpeed * 4248);
	SmartDashboard::PutNumber("Back Right Theoretical Speed", brSpeed * 4248);
	SmartDashboard::PutNumber("Back Left Theoretical Speed", blSpeed * 4248);

	SmartDashboard::PutNumber("Front Right Theoretical Angle", frAngle);
	SmartDashboard::PutNumber("Front Left Theoretical Angle", flAngle);
	SmartDashboard::PutNumber("Back Right Theoretical Angle", brAngle);
	SmartDashboard::PutNumber("Back Left Theoretical Angle", blAngle);

	a_FrontRight.Update(frAngle, frSpeed, FRONT_RIGHT_TURN_OFFSET, gyroValue);
	a_FrontLeft.Update(flAngle, flSpeed, FRONT_LEFT_TURN_OFFSET, gyroValue);
	a_BackLeft.Update(blAngle, blSpeed, BACK_LEFT_TURN_OFFSET, gyroValue);
	a_BackRight.Update(brAngle, brSpeed, BACK_RIGHT_TURN_OFFSET, gyroValue);
}

float SwerveDrive::GetDistanceY() // on a roughly square robot an average of all distances traveled should roughly be the distance the c.o.m. traveled
{
	return (a_FrontRight.GetDistanceY() + a_BackRight.GetDistanceY() + a_FrontLeft.GetDistanceY() + a_BackLeft.GetDistanceY()) / 4;
}

float SwerveDrive::GetDistanceX()
{
	return (a_FrontRight.GetDistanceX() + a_BackRight.GetDistanceX() + a_FrontLeft.GetDistanceX() + a_BackLeft.GetDistanceX()) / 4;
}

float SwerveDrive::GetDistanceRightX()
{
	return (a_FrontRight.GetDistanceX() + a_BackRight.GetDistanceX()) / 2;
}
float SwerveDrive::GetDistanceLeftX()
{
	return (a_FrontLeft.GetDistanceX() + a_BackLeft.GetDistanceX()) / 2;
}
float SwerveDrive::GetDistanceRightY()
{
	return (a_FrontRight.GetDistanceY() + a_BackRight.GetDistanceY()) / 2;
}
float SwerveDrive::GetDistanceLeftY()
{
	return (a_FrontLeft.GetDistanceY() + a_BackLeft.GetDistanceY()) / 2;
}

void SwerveDrive::Zero()
{
	a_FrontRight.ResetDistances();
	a_FrontLeft.ResetDistances();
	a_BackLeft.ResetDistances();
	a_BackRight.ResetDistances();
}

void SwerveDrive::SetTwistingMode()
{
	isTwisting = true;
}

void SwerveDrive::DisableTwist()
{
	isTwisting = false;
}

void SwerveDrive::SetTwistingRelAngle(float gyroAngle, float angle) // only call once, please, or the robot will eternally spin
{
	turnAngle = gyroAngle + angle;
}
