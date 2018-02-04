#ifndef COMPPREFS_H
#define COMPPREFS_H

#define FRONT_RIGHT_TURN_OFFSET 0
#define FRONT_LEFT_TURN_OFFSET 0
#define BACK_RIGHT_TURN_OFFSET 180
#define BACK_LEFT_TURN_OFFSET 0

#define FRONT_RIGHT_TURN_PID 20,0,75
#define FRONT_RIGHT_DRIVE_PIDF 0.75,0.00005,0,1.0
#define FRONT_RIGHT_IZONE 0
#define FRONT_LEFT_TURN_PID 20,0,75
#define FRONT_LEFT_DRIVE_PIDF 0.75,0.00005,0,1.0
#define FRONT_LEFTT_IZONE 0
#define BACK_RIGHT_TURN_PID 16.5,0,50
#define BACK_RIGHT_DRIVE_PIDF 1.5,0.001,0,1.0
#define BACK_RIGHT_IZONE 0
#define BACK_LEFT_TURN_PID 15,0,150
#define BACK_LEFT_DRIVE_PIDF 1,0.0005,0,1.0
#define BACK_LEFT_IZONE 0

#define SHOOTER_P 150.0
#define SHOOTER_I 0.5
#define SHOOTER_D 0.5
#define SHOOTER_F 1.0
#define SHOOTER_IZONE 0

#define TURN_ENCODER_TYPE FeedbackDevice::PulseWidthEncodedPosition
#define ABSOLUTE_CONV_FACTOR (1 / 360.0)

#define PREFS_FUNCTIONS a_BackRight.InvertDriveMotor(); a_FrontRight.InvertDriveMotor();

#endif
