/* Teleoperation System for Mobile Manipulators Framework
 *
 * Copyright (C) 2015 
 * RECONFIGURABLE CONTROL OF ROBOTIC SYSTEMS OVER NETWORKS Project
 * Robotics and Control Systems Laboratory
 * Department of Electrical Engineering
 * Sapientia Hungarian University of Transylvania
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * For more details see the project homepage:
 * <http://www.ms.sapientia.ro/~martonl/MartonL_Research_TE.htm>
 */


#ifndef YOUBOT_H
#define YOUBOT_H

#include "Robot.h"
#include "Motor.h"
#include "Log.h"

using namespace VrepAPI;

class YouBot
	: public Robot
{
public:
	YouBot(int handle, std::string name);
	YouBot(std::string name);

	~YouBot();

	// TODO: remove
	void Move(double vx, double vy, double wz);
	
	// Load motor from V-REP
	virtual void Load(VrepConnectorPtr& vrepConnectorPtr);

	// Write and read values to V-REP server.
	virtual void Update(VrepConnectorPtr& vrepConnectorPtr);

	// Set the wheel speed from give v, w, wz
	void SetWheelSpeed(double vx, double vy, double wz);
	
protected:
	Utils::LogPtr m_Logger;

	// Calculates the speed if no reference is given
	Vector2 CalculateSpeed();

	// Transforms the robot speed = (linear, angular) in to wheel speed
	void SetWheelSpeed(Vector2 robotSpeed);

	// Claculates the speed from given vx, vy, wz
	void CalculateWheelSpeed(double hapticPosZ, double hapticGimbalAngle, double hapticPosX);

private:

	enum YouBotJoint
	{
		RollingJoint_RearRight,
		RollingJoint_RearLeft,
		RollingJoint_FrontRight,
		RollingJoint_FrontLeft,
		ArmJoint0,
		ArmJoint1,
		ArmJoint2,
		ArmJoint3,
		ArmJoint4,
		GripperJoint1,
		GripperJoint2,
	};

	std::map<YouBotJoint, MotorPtr> m_Motors;

	// The linear velocity in V-REP: vx, vy, vz. In global reference frame
	std::vector<double> m_VelocityRawLinear;

	// The angular velocity in V-REP: wx, wy, wz. In global reference frame
	std::vector<double> m_VelocityRawAngular;

	double const m_HalfAxeLength;
	double const m_WheelRadius;
	double const m_HalfFrontAxeLength;

	// TODO: why are these needed 
	double m_VelocityFrontLeft;
	double m_VelocityFrontRight;
	double m_VelocityRearLeft;
	double m_VelocityRearRight;

	double HAPTIC_X_MIN = -70;
	double HAPTIC_X_MAX = 70;
	double HAPTIC_Y_MIN = -100;
	double HAPTIC_Y_MAX = 100;
	double HAPTIC_G1_MIN = -1.4;
	double HAPTIC_G1_MAX = 1.4;

};

#endif // YouBot_H