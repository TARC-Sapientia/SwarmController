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


#include "VrepObject.h"
#include "VrepConnector.h"

using namespace VrepAPI;

VrepObject::VrepObject(std::string name)
	: VrepObject(-1, name)
{
}

VrepObject::VrepObject(int handle, std::string name)
	: m_Handle(handle),
	m_Name(name),
	m_Position(3, 0.0),
	m_Orientation(3, 0.0),
	m_IsStreamingInitialized(false)
{
}

VrepObject::VrepObject(const VrepObject& other)
	: m_Handle(other.GetHandle()),
	m_Name(other.GetName()),
	m_Position(other.GetPosition()),
	m_Orientation(other.GetOrientation()),
	m_IsStreamingInitialized(other.IsStreamingInitialized())
{
}

VrepObject::~VrepObject()
{
}


int VrepObject::GetHandle() const
{
	return m_Handle;
}

std::string VrepObject::GetName() const
{
	return m_Name;
}

std::vector<double> VrepObject::GetPosition() const
{
	return m_Position;
}

std::vector<double> VrepObject::GetOrientation() const
{
	return m_Orientation;
}

bool VrepObject::IsStreamingInitialized() const
{
	return m_IsStreamingInitialized;
}