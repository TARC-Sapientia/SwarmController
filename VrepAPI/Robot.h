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


#ifndef ROBOT_H
#define ROBOT_H

#include "VrepComplexObject.h"
#include "Vector2.h"

namespace VrepAPI
{
	// An abstract robot interface.
	// Note: speed is a scalar value, velocity is a vector.
	// Vector2 class is used to store both speed and velocity.
	class Robot :
		public VrepComplexObject
	{
	public:
		Robot(std::string name);
		Robot(int handle, std::string name);

		virtual ~Robot();

		// Set the reference speed for the robot. This function modifies the speed dirty flag
		// velocityRef is a (referenceLinearSpeed, referenceAngularSpeed) pair
		// Speed is a scalar value.
		void SetSpeed(Vector2 speedReference);

		// Get the current velocity of the robot in V-REP: v = (vx, vy). The z axis is ignored 		
		Vector2 GetLinearVelocity();

		bool m_IsInformed;

		// Set if the robot is informed or not
		// By default robots are not informed
		void SetIsInformed(bool isInformed);

		// See if the robot is informed
		bool IsInformed();

	protected:
		
		// Maximum speed: x = linear [m/s], y = angular [rad/s])
		Vector2 m_SpeedMaximum;

		// Minimum speed: x = linear [m/s], y = angular [rad/s])
		Vector2 m_SpeedMinimum;

		// The linear velocity in V-REP: vx, vy, vz. In global reference frame
		std::vector<double> m_VelocityRawLinear;

		// The angular velocity in V-REP: wx, wy, wz. In global reference frame
		std::vector<double> m_VelocityRawAngular;

		// Flag indicating the robot speed status
		// If true the robot is given a reference speed
		bool m_IsSpeedDirty;

		// The reference speed = (referenceLinearSpeed, referenceAngularSpeed)
		Vector2 m_SpeedReference;

		// Calculates the speed if no reference is given
		virtual Vector2 CalculateSpeed() = 0;
		
		// Transforms the robot speed = (linear, angular) in to wheel speed
		virtual void SetWheelSpeed(Vector2 robotSpeed) = 0;
	};

	typedef boost::shared_ptr<Robot> RobotPtr;
}

#endif // ROBOT_H