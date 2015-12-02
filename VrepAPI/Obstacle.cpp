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


#include "Obstacle.h"

using namespace VrepAPI;

Obstacle::Obstacle(int handle, std::string name)
	: VrepSimpleObject(handle, name)
{
}

Obstacle::Obstacle(std::string name)
	: Obstacle(-1, name)
{
}

Obstacle::~Obstacle()
{
}

void Obstacle::Load(VrepConnectorPtr& vrepConnectorPtr)
{
}

void Obstacle::Update(VrepConnectorPtr& vrepConnectorPtr)
{
	m_Position = vrepConnectorPtr->GetObjectPosition(m_Handle, -1, m_IsStreamingInitialized);
	m_Orientation = vrepConnectorPtr->GetObjectOrientation(m_Handle, -1, m_IsStreamingInitialized);

	if (m_IsStreamingInitialized == false) m_IsStreamingInitialized = true;
}