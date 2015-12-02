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


#include "Motor.h"
#include "VrepConnector.h"
#define _USE_MATH_DEFINES
#include <math.h>
#include <iostream>

using namespace VrepAPI;

Motor::Motor(int handle, std::string name)
	: VrepSimpleObject(handle, name),
	m_PositionMin(0.0),
	m_PositionMax(0.0),
	m_Position(0.0),
	m_Torque(0.0),
	m_PositionReference(0.0),
	m_SpeedReference(0.0),
	m_IsPositionDirty(false),
	m_IsSpeedDirty(false),
	m_TorqueMax(500.0)
{
}

Motor::Motor(std::string name)
	: Motor(-1, name)
{
}

Motor::~Motor()
{
}

double Motor::GetPositionMin() const
{
	return m_PositionMin;
}

double Motor::GetPositionMax() const
{
	return m_PositionMax;
}

double Motor::GetPosition() const
{
	return m_Position;
}

void Motor::SetPosition(double positionReference)
{
	m_PositionReference = positionReference;
	m_IsPositionDirty = true;
}

void Motor::SetPositionInDegree(double positionReference)
{
	SetPosition(positionReference * M_PI / 180);
}

void Motor::SetSpeed(double speedReference)
{
	m_SpeedReference = speedReference;
	m_IsSpeedDirty = true;
}

double Motor::GetTorqueMax() const
{
	return m_TorqueMax;
}

double Motor::GetTorque() const
{
	return m_Torque;
}

void Motor::Load(VrepConnectorPtr& vrepConnectorPtr)
{
}

void Motor::Update(VrepConnectorPtr& vrepConnectorPtr)
{
	if (m_IsSpeedDirty == true)
	{
		m_IsSpeedDirty = false;

		vrepConnectorPtr->SetMotorVelocity(m_Handle, m_SpeedReference);
	}

	if (m_IsStreamingInitialized == false) m_IsStreamingInitialized = true;
}