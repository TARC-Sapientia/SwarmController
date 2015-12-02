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


#include "EpuckProximitySensor.h"
#include "Epuck.h"
#include "Utilities.h"
#include <string>
#include <math.h>

using namespace std;
using namespace Utils;
using namespace VrepAPI;

EpuckProximitySensor::EpuckProximitySensor(std::string name)
	: EpuckProximitySensor(-1, name)
{
}

EpuckProximitySensor::EpuckProximitySensor(int handle, std::string name)
	: VrepSimpleObject(handle, name)
{
	// The names must match those found in the simulator	
	if (name.find("proxSensor1") != string::npos) m_AngleOffset = -90.0;
	if (name.find("proxSensor2") != string::npos) m_AngleOffset = -45.85987261;
	if (name.find("proxSensor3") != string::npos) m_AngleOffset = -17.19745223;
	if (name.find("proxSensor4") != string::npos) m_AngleOffset = 17.19745223;
	if (name.find("proxSensor5") != string::npos) m_AngleOffset = 45.85987261;;
	if (name.find("proxSensor6") != string::npos) m_AngleOffset = 90.0;
	if (name.find("proxSensor7") != string::npos) m_AngleOffset = 151.3375796;
	if (name.find("proxSensor8") != string::npos) m_AngleOffset = -151.3375796;

	// Calculate these values only once
	m_AngleOffsetCos = cos(DegreeToRadian(m_AngleOffset));
	m_AngleOffsetSin = sin(DegreeToRadian(m_AngleOffset));

	// Get the parent object
	if (auto parentPtr = GetParent().lock())
	{
		// Downcast to an epuck
		if (auto epuck = boost::dynamic_pointer_cast<Epuck>(parentPtr))
		{
			// Get retrieve the radius
			m_DistanceOffset = epuck->GetRadius();
		}
	}
	else
	{
		m_DistanceOffset = 0.0;
	}
}

EpuckProximitySensor::~EpuckProximitySensor()
{
}

void EpuckProximitySensor::SetData(const VrepObjectWeakPtr& rawObject, std::vector<double> point)
{
	m_IsDetecting = true;

	m_DetectedObject = rawObject;

	// The detected object is in the proximity sensors reference frame
	// The coordinates, x = front, y = sideways, z = ignored
	// We ignore the height represented by the z axis
	double xp = point[2]; // x - in proxSensor reference frame
	double yp = point[1]; // y - in proxSensor reference frame

	// (xe,xe) the epuck reference frame
	// theta the rotation angle from proximity sensor to epuck reference frame (x axis)
	// (xp,xp) the proximity sensor reference frame

	// Rotate the detected point to the epuck reference frame, and perform the radius translation. The formula: 
	//double xe = radius * cos(theta) + xp * cos(theta) - yp * sin(theta);
	//double ye = radius * sin(theta) + xp * sin(theta) + yp * cos(theta);
	double xe = m_DistanceOffset * m_AngleOffsetCos + xp * m_AngleOffsetCos - yp * m_AngleOffsetSin;
	double ye = m_DistanceOffset * m_AngleOffsetSin + xp * m_AngleOffsetSin + yp * m_AngleOffsetCos;

	//cout << "  xe: " << xe << '\t' << "ye: " << ye << endl;

	m_DetectedPoint = Vector2(xe, ye); 

	m_DetectedAngle = -RadianToDegree(atan2(ye, xe));

	// From robot' center to detected point: magnitude(detectedPoint - (0,0)) 
	m_DetectedDistance = m_DetectedPoint.Magnitude();
}

void EpuckProximitySensor::SetDetectionStatus(bool isDetecting)
{
	m_IsDetecting = isDetecting;
}

bool EpuckProximitySensor::IsDetecting() const 
{
	return m_IsDetecting;
}

Vector2 EpuckProximitySensor::GetPoint() const
{
	return m_DetectedPoint;
}

double EpuckProximitySensor::GetDistance() const
{
	return m_DetectedDistance;
}

double EpuckProximitySensor::GetAngle() const
{
	return m_DetectedAngle;
}

const VrepObjectWeakPtr& EpuckProximitySensor::GetObjectWeakptr() const
{
	return m_DetectedObject;
}
