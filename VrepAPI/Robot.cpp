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


#include "Robot.h"

using namespace VrepAPI;

Robot::Robot(std::string name)
	: VrepComplexObject(-1, name),
	m_IsInformed(true)
{
}

Robot::Robot(int handle, std::string name)
	: VrepComplexObject(handle, name),
	m_VelocityRawLinear(3, 0.0),
	m_VelocityRawAngular(3, 0.0),
	m_IsSpeedDirty(false),
	m_IsInformed(true)
{
}

Robot::~Robot()
{
}

void Robot::SetSpeed(Vector2 speedReference)
{
	m_SpeedReference = speedReference;

	if (m_SpeedReference.x > m_SpeedMaximum.x) m_SpeedReference.x = m_SpeedMaximum.x;
	if (m_SpeedReference.x < m_SpeedMinimum.x) m_SpeedReference.x = m_SpeedMinimum.x;

	// m_SpeedReference.y => w 
	auto beta = m_Orientation[1];
	auto Kbeta = 0.1;
	auto betaRef = 0.0;

	m_SpeedReference.y = Kbeta * (betaRef - beta);

	m_IsSpeedDirty = true;
}

Vector2 Robot::GetLinearVelocity()
{
	return Vector2(m_VelocityRawLinear[0], m_VelocityRawLinear[1]);
}

void Robot::SetIsInformed(bool isInformed)
{
	this->m_IsInformed = isInformed;
}

bool Robot::IsInformed()
{
	return this->m_IsInformed;
}