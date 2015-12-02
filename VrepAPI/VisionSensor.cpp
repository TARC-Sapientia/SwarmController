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


#include "VisionSensor.h"
#include "VrepConnector.h"
#include <iostream>

using namespace std;
using namespace VrepAPI;

VisionSensor::VisionSensor(int handle, std::string name)
	: VrepSimpleObject(handle, name)
{
}

VisionSensor::VisionSensor(std::string name)
	: VisionSensor(-1, name)
{
}

VisionSensor::~VisionSensor()
{
}

void VisionSensor::Load(VrepConnectorPtr& vrepConnectorPtr)
{
	m_Handle = vrepConnectorPtr->GetObjectHandle(m_Name);
}

void VisionSensor::Update(VrepConnectorPtr& vrepConnectorPtr)
{
	// TODO: wtf is options?!
	// options: image options, bit-coded:
	// bit0 set : each image pixel is a byte(greyscale image), otherwise each image pixel is a rgb byte - triplet
	char options = 4; 
	
	//m_ImageArrayMutex.lock();
	vrepConnectorPtr->GetVisionSensorData(m_Handle, m_ImageHeight, m_ImageWidht, m_ImageArray, options, m_IsStreamingInitialized);
	//m_ImageArrayMutex.unlock();

	m_IsStreamingInitialized = true;
}

std::vector<unsigned char>& VisionSensor::GetImageArray()
{
	// Return a copy of the current frame
	//m_ImageArrayMutex.lock();
	//auto imageArray = std::vector<unsigned char>(m_ImageArray);
	//m_ImageArrayMutex.unlock();

	//return imageArray;

	return m_ImageArray;
}

int VisionSensor::GetImageHeight()
{
	return m_ImageWidht;
}

int VisionSensor::GetImageWidth()
{
	return m_ImageHeight;
}