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


#include "ProximitySensor.h"
#include "VrepConnector.h"

using namespace VrepAPI;

ProximitySensor::ProximitySensor(std::string name)
	: ProximitySensor(-1, name)
{
}

ProximitySensor::ProximitySensor(int handle, std::string name)
	: VrepSimpleObject(handle, name),
	m_DetectionsState(false),
	m_DetectedPoint(3, 0.0),
	m_DetectedSurfaceNormalVector(3, 0.0),
	m_DetectedObjectHandle(-1)
{
}

ProximitySensor::~ProximitySensor()
{
}

void ProximitySensor::Load(VrepConnectorPtr& vrepConnectorPtr)
{
}

void ProximitySensor::Update(VrepConnectorPtr& vrepConnectorPtr)
{
	m_DetectionsState = false;

	m_DetectedPoint.clear();
	m_DetectedSurfaceNormalVector.clear();
	m_DetectedObjectHandle = -1;

	vrepConnectorPtr->GetProximitySensorData(m_Handle, m_DetectionsState, m_DetectedPoint, m_DetectedObjectHandle, m_DetectedSurfaceNormalVector, m_IsStreamingInitialized);

	m_IsStreamingInitialized = true;
}

bool ProximitySensor::IsDetecting()
{
	return m_DetectionsState;
}

int ProximitySensor::GetDetectedObjectHandle()
{
	return m_DetectedObjectHandle;
}

std::vector<double> ProximitySensor::GetDetectedPoint()
{
	return m_DetectedPoint;
}

std::vector<double> ProximitySensor::GetDetectedNormalVector()
{
	return m_DetectedSurfaceNormalVector;
}