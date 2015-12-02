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


#include "Epuck.h"
#include "Motor.h"
#include "Utilities.h"
#include "Obstacle.h"
#include <string>
#include <iostream>
#include <math.h>
#include <chrono>

using namespace std;
using namespace Utils;
using namespace VrepAPI;

#ifdef USE_IMAGEPROCESSOR
using namespace ImageProcessing;
#endif

Epuck::Epuck(std::string name)
	: Epuck(-1, name)
{
}

Epuck::Epuck(int handle, std::string name)
	: Robot(handle, name),
	m_HalfAxleLength(0.052 / 2.0),
	m_WheelRadius(0.0205),
	m_Radius(0.037),
	m_DetectionDistanceMin(50.0),
	m_DetectionDistanceMax(450.0),
	m_ForceMaximum(100.0),
	m_ForceMinimum(10.0),
	m_Logger(new Log("Log", name)),
	m_RobotControl()
{
	m_SpeedMaximum.x = 0.3; // [m/s]
	m_SpeedMaximum.y = 0.0; // [rad/s]

	m_SpeedMinimum.x = 0.05; // [m/s]
	m_SpeedMinimum.y = 0.0; // [rad/s]
	
	m_ProximitySensors.reserve(9);

	#ifdef USE_IMAGEPROCESSOR
	m_ImageProcessorPtr.reset(new ImageProcessor());
	#endif
}

Epuck::~Epuck()
{
}

void Epuck::Load(VrepConnectorPtr& vrepConnectorPtr)
{
	m_MotorLeftPtr = boost::dynamic_pointer_cast<Motor>(GetComponent("leftJoint"));
	m_MotorRightPtr = boost::dynamic_pointer_cast<Motor>(GetComponent("rightJoint"));
	string camera("ePuck_camera");

	for (auto component : m_Components)
	{
		if (auto proximitySensor = boost::dynamic_pointer_cast<EpuckProximitySensor>(component))
		{
			// Add proximity sensors
			m_ProximitySensors.push_back(proximitySensor);
		}

		if (auto visionSensor = boost::dynamic_pointer_cast<VisionSensor>(component))
		{
			// Add vision sensors
			if (visionSensor->GetName().find(camera) != std::string::npos)
			{
				m_VisionSensorPtr = visionSensor;
			}
		}
	
		// Load each component
		boost::dynamic_pointer_cast<VrepObject>(component)->Load(vrepConnectorPtr);
	}
}

void Epuck::Update(VrepConnectorPtr& vrepConnectorPtr)
{	
	// Update position and orientation
	m_Position = vrepConnectorPtr->GetObjectPosition(m_Handle, -1, m_IsStreamingInitialized);
	m_Orientation = vrepConnectorPtr->GetObjectOrientation(m_Handle, -1, m_IsStreamingInitialized);

	// Update velocity
	vrepConnectorPtr->GetObjectVelocity(m_Handle, m_VelocityRawLinear, m_VelocityRawAngular, m_IsStreamingInitialized);

	#ifdef USE_IMAGEPROCESSOR

	// Look for nearby objects
	ProcessProximitySensors();

	// Read and process the image only if the robot is informed
	if (this->IsInformed())
	{
		// Update Vision Sensor data
		m_VisionSensorPtr->Update(vrepConnectorPtr);

		// Process the frame
		auto detectedPointList = m_ImageProcessorPtr->ProcessImage(m_VisionSensorPtr->GetImageArray(),
			m_VisionSensorPtr->GetImageHeight(),
			m_VisionSensorPtr->GetImageWidth(),
			false);

		if (m_ImageProcessorPtr->IsDetecting())
		{
			// Set the detected point list from image
			m_RobotControl.SetDetectedPoints(detectedPointList);

			// Set the proximity snsor force
			m_RobotControl.SetProximitySensorForce(GetProximitySensorForce());

			m_SpeedReference = m_RobotControl.CalculateSpeed();

			// Assign wheel speed
			SetWheelSpeed(m_SpeedReference);
		}
		else
		{
			if (m_IsSpeedDirty == false)
			{
				// Calculate velocity
				m_SpeedReference = CalculateSpeed();
			
				SetWheelSpeed(m_SpeedReference);
			}
			else
			{
				// The robot is given a reference velocity. Reset the flag
				m_IsSpeedDirty = false;
			}
		}
	} 
	else 
	{
		if (m_IsSpeedDirty == false)
		{
			// Calculate velocity
			m_SpeedReference = CalculateSpeed();

			SetWheelSpeed(m_SpeedReference);
		}
		else
		{
			// The robot is given a reference velocity. Reset the flag
			m_IsSpeedDirty = false;
		}
	}

	

	#else
	if (m_IsSpeedDirty == false)
	{
		// Look for nearby objects
		ProcessProximitySensors();

		// Calculate velocity
		m_SpeedReference = CalculateSpeed();
	}
	else
	{
		// The robot is given a reference velocity. Reset the flag
		m_IsSpeedDirty = false;
	}
	
	// Assign wheel speed
	SetWheelSpeed(m_SpeedReference);//(Vector2(0, 0));
	#endif

	if (m_IsStreamingInitialized == true)
	{
		// Log status
		m_Logger->Write(
			"v: " + to_string(GetLinearVelocity().Magnitude()) + "\tw: 0.000"
			+ "\tvRef: " + to_string(m_SpeedReference.x) + "\twRef: " + to_string(m_SpeedReference.y)
			+ "\tXg: " + to_string(m_Position[0]) + "\tYg: " + to_string(m_Position[1]) + "\tZg: " + to_string(m_Position[2])
			+ "\tAlpha: " + to_string(m_Orientation[0]) + "\tBeta: " + to_string(m_Orientation[1]) + "\tGamma: " + to_string(m_Orientation[2])
			+ "\n");
	}

	// Update motors
	m_MotorLeftPtr->Update(vrepConnectorPtr);
	m_MotorRightPtr->Update(vrepConnectorPtr);

	if (m_IsStreamingInitialized == false) m_IsStreamingInitialized = true;
}

void Epuck::ProcessProximitySensors()
{
	for (auto& curSensor : m_ProximitySensors)
	{
		if (curSensor->IsDetecting() == true)
		{
			// The proximity sensor only holds a weak_ptr and it should be available at this moment
			if (auto curObject = curSensor->GetObjectWeakptr().lock())
			{
				// We are detecting an object. Must check if an other sensor is detecting the same object
				for (auto& otherSensor : m_ProximitySensors)
				{
					// If a different sensor is also detecting
					if (otherSensor != curSensor && otherSensor->IsDetecting())
					{
						// Grab the detected object
						if (auto otherObject = otherSensor->GetObjectWeakptr().lock())
						{
							// Check if other sensor is detecting the same object
							if (curObject == otherObject)
							{
								// Only the closest detection is considered the other is ignored
								if (curSensor->GetDistance() < otherSensor->GetDistance())
								{
									otherSensor->SetDetectionStatus(false);
								}
								else
								{
									curSensor->SetDetectionStatus(false);
								}
							}
						}
					}
				}
			}
		}
	}
}

Vector2 Epuck::GetProximitySensorForce()
{
	Vector2 fSumPull;
	Vector2 fSumPush;

	for (auto& proxSensor : m_ProximitySensors)
	{
		Vector2 fPull;
		Vector2 fPush;

		if (proxSensor->IsDetecting() == true)
		{
			if (auto epuck = boost::dynamic_pointer_cast<Epuck>(proxSensor->GetObjectWeakptr().lock()))
			{
				// Calculate pull only if an epuck is detected
				fPull = proxSensor->GetPoint();
				fPull.SetMagnitude(CalculateForcePull(fPull.Magnitude()));
			}

			// Calculate push 
			fPush = proxSensor->GetPoint().Invert();
			fPush.SetMagnitude(CalculateForcePush(fPush.Magnitude()));

			// Add to sum
			fSumPull += fPull;
			fSumPush += fPush;

			// Reset the detection flag
			proxSensor->SetDetectionStatus(false);
		}
	}

	double kPull = 1;// 0.5;
	double kPush = 1;// 0.5;

	Vector2 sum = (kPull * fSumPull) + (kPush * fSumPush);

	return sum;
}

Vector2 Epuck::CalculateSpeed()
{
	double v = 0.0;
	double w = 0.0;
	Vector2 sum = GetProximitySensorForce();

	double kw = 0.00275;

	// The owerall sum should not exceed speed limit
	sum.Clamp(m_ForceMaximum);
	
	// Calculate linear and angular speed
	double d = sum.Magnitude();
	if (d > m_ForceMinimum)
	{
		double m = (m_SpeedMaximum.x - m_SpeedMinimum.x) / (m_ForceMaximum - m_ForceMinimum);
		double n = m_SpeedMinimum.x - m * m_ForceMinimum;

		v = d * m + n;
		w = kw * (int)(0.5 + sum.Angle()); // ceil and floor
	}

	//cout
		//<< "\tPull: " << fSumPull.Magnitude()
		//<< "\tPush: " << fSumPush.Magnitude()
		//<< "\tSum: " << sum.Magnitude()
		//<< "\tv: " << v
		//<< "\tw: " << w
		//<< "\ta: " << (int)(0.5 + sum.Angle())
		//<< endl;

	//return Vector2(0.0, 0.0);
	return Vector2(v, w);
}

double Epuck::CalculateForcePush(double distance)
{
	double force = 0.0;
	
	// Convert from [m] to [mm] to obtain a number greater than 1. Otherwise the gauss will not work
	distance *= 1000;

	if (distance > m_DetectionDistanceMax) distance = m_DetectionDistanceMax;
	if (distance < m_DetectionDistanceMin) distance = m_DetectionDistanceMin;
	
	const double GAUSS_CUT_OFF_VALUE = m_ForceMinimum;
	const double GAUSS_HEIGHT_OF_CURVE_PEAK = m_ForceMaximum;
	const double GAUSS_WIDTH_OF_CURVE = 50.0;

	force = GAUSS_HEIGHT_OF_CURVE_PEAK * pow(M_E, (-(pow(distance - m_DetectionDistanceMin, 2) / (2 * pow(GAUSS_WIDTH_OF_CURVE, 2)))));

	if (force < GAUSS_CUT_OFF_VALUE)
	{
		return 0.0;
	}

	return force;
}

double Epuck::CalculateForcePull(double distance)
{
	double force = 0.0;

	// Convert from [m] to [mm] to obtain a number greater than 1. Otherwise the gauss will not work
	distance *= 1000;

	if (distance > m_DetectionDistanceMax) distance = m_DetectionDistanceMax;
	if (distance < m_DetectionDistanceMin) distance = m_DetectionDistanceMin;
	
	const double GAUSS_CUT_OFF_VALUE = m_ForceMinimum;
	const double GAUSS_HEIGHT_OF_CURVE_PEAK = m_ForceMaximum;
	const double GAUSS_WIDTH_OF_CURVE = 100.0;

	force = GAUSS_HEIGHT_OF_CURVE_PEAK * pow(M_E, (-(pow(m_DetectionDistanceMax - distance, 2) / (2 * pow(GAUSS_WIDTH_OF_CURVE, 2)))));
	
	if (force < GAUSS_CUT_OFF_VALUE)
	{
		return 0.0;
	}

	return force;
}

void Epuck::SetWheelSpeed(Vector2 robotSpeed)
{
	// In theory this should work. 
    // double div = (m_WheelRadius / 2) * (1 + (1 / m_HalfAxleLength)); // = 0.404..;
	
	// However it does not include the start PWM. So we use this value instead:
	double div = 0.0211;

	m_MotorLeftPtr->SetSpeed((robotSpeed.x - robotSpeed.y) / div);
	m_MotorRightPtr->SetSpeed((robotSpeed.x + robotSpeed.y) / div);
}

double Epuck::GetWheelRadius() const
{
	return m_WheelRadius;
}

double Epuck::GetRadius() const
{
	return m_Radius;
}