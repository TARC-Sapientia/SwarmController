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


#include "VrepGeneralObject.h"

using namespace VrepAPI;

VrepGeneralObject::VrepGeneralObject(std::string name)
	: VrepSimpleObject(-1, name)
{
}

VrepGeneralObject::VrepGeneralObject(int handle, std::string name)
	: VrepSimpleObject(handle, name)
{
}

VrepGeneralObject::~VrepGeneralObject()
{
}

void VrepGeneralObject::Load(VrepConnectorPtr& vrepConnectorPtr)
{
}

void VrepGeneralObject::Update(VrepConnectorPtr& vrepConnectorPtr)
{
}