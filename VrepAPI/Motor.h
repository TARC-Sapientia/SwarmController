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


#ifndef MOTOR_H
#define MOTOR_H

#include "VrepSimpleObject.h"

namespace VrepAPI
{
	// Represents V-REP Joint Object
	class Motor
		: public VrepSimpleObject
	{
	public:
		// Initialize the motor with the given V-REP handle
		Motor(int handle, std::string name);
		Motor(std::string name);

		virtual ~Motor();

		double GetPositionMin() const;
		double GetPositionMax() const;

		// Return the current position of the joint (radian)
		double GetPosition() const;

		// The reference position for the joint
		void SetPosition(double positionReference);

		// The reference position in degree for the joint
		void SetPositionInDegree(double positionReference);

		// The reference speed for the joint
		void SetSpeed(double speedReference);

		double GetTorqueMax() const;

		// Return the current force/torque of the joint
		double GetTorque() const;

		// Load motor from V-REP
		virtual void Load(VrepConnectorPtr& vrepConnectorPtr);

		// Write and read values to V-REP server.
		virtual void Update(VrepConnectorPtr& vrepConnectorPtr);

	private:

		// Minimum position in radian
		const double m_PositionMin;

		// Maximum position in radian
		const double m_PositionMax;

		// Current position of the join from V-REP sensor (radian)
		double m_Position;

		// Current force/torque of the join from V-REP sensor
		double m_Torque;

		// Speed reference to be sent to the joint
		double m_SpeedReference;

		// Position reference to be sent to the joint
		double m_PositionReference;

		// If true a reference position was given 
		bool m_IsPositionDirty;

		// If true a reference speed was given 
		bool m_IsSpeedDirty;

		// The maximum value of torque apply on the joint
		const double m_TorqueMax;
	};

	typedef boost::shared_ptr<Motor> MotorPtr;
}

#endif // MOTOR_H