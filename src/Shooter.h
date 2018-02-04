/*
 * Shooter.h
 *
 *  Created on: Sep 16, 2017
 *      Author: Alexis
 */

#ifndef SRC_SHOOTER_H_
#define SRC_SHOOTER_H_

#include <WPILib.h>
#include "ctre/Phoenix.h"
#include "Prefs.h"

class Shooter // Simple PIDF flywheel shooter
{
public:
	Shooter(int firePort);
	virtual ~Shooter() = default;

	void Set(float speed);

	float GetSpeed();

	void InvertQuad();

	void InvertFlyWheelMotor();

	void SetWheelPIDF(float wheelP, float wheelI, float wheelD, float wheelF);
private:
	WPI_TalonSRX a_ShooterTennis;
};

#endif /* SRC_SHOOTER_H_ */
