/*
 *
 * Let's see if this actually works.
 *
 */

#ifndef SRC_TENNISBOT_H_
#define SRC_TENNISBOT_H_

#include <WPILib.h>
#include <SerialPort.h>
#include <IterativeRobot.h>

#include "JrimmyGyro.h"
#include "SwerveModule.h"
#include "SwerveDrive.h"
#include "Shooter.h"
#include "LightRingController.h"
#include "SmartDashboard/SmartDashboard.h"
#include "SmartDashboard/SendableChooser.h"

class TennisBot : public IterativeRobot {
public:
	TennisBot(void);
	void RobotInit(void);
	void RobotPeriodic(void);
	void DisabledInit(void);
	void DisabledPeriodic(void);
	void TeleopInit(void);
	void TeleopPeriodic(void);
	void TestInit(void);
	void TestPeriodic(void);
	void AutonomousInit(void);
	void AutonomousPeriodicFull(void);
	void AutonomousPeriodic(void);
	void AutonomousPeriodicSimple(void);

private:
	Joystick a_Joystick;
	Joystick a_Joystick2;

	PowerDistributionPanel a_PDP;

	SwerveModule a_FrontRight;
	SwerveModule a_FrontLeft;
	SwerveModule a_BackLeft;
	SwerveModule a_BackRight;

	SwerveDrive a_Drive;

	Shooter a_Shooter;

	LightRingController a_LRC;

	ADXL345_I2C a_Accelerometer;

	JrimmyGyro a_Gyro;
};

#endif
