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


#ifndef ROBOTCONTROL_H
#define ROBOTCONTROL_H

#include "Vector2.h"
#include "ImageProcessor.h"
#include <vector>
#include <iostream>

using namespace std;

namespace VrepAPI
{
	class RobotControl
	{
	public:
		// Constructor
		RobotControl();
		// Calculate robot speed
		Vector2 CalculateSpeed();
		// Set detected point list
		void SetDetectedPoints(map<int, map<string, double>> pointList);
		// Set proximity sensor calculated force
		void SetProximitySensorForce(Vector2 force);

	private:
		// Get weights for speed calculation
		void GetWeights();
		// Get the maximum detected blob size
		void GetMaxBlobSize();
		// Set the default weights
		void SetDefaultWeights();

		// Detected point list
		map<int, map<string, double>> m_DetectedPointList;
		// Calculated force
		Vector2 m_Force;
		// Calculated force for camera
		Vector2 m_CameraFroce;
		// Proximity sensor fore
		Vector2 m_ProximitySensorForce;
		// Maximum detected blob size
		double m_MaxBlobSize;
		// Weights for speed calculation
		map<int, double> m_Weights;
		// Trashold
		const double m_Trashold;
		// Maximum speed
		const double m_MaxSpeed;
		//Minimum speed
		const double m_MinSpeed;
		// Constant KW
		const double m_KW;
		//Maximum speed
		Vector2 m_SpeedMaximum;
		// Minimum speed
		Vector2 m_SpeedMinimum;
		// See if robot is close enough to the Goal
		map<int, bool> m_IsCloseEnough;
		// Set the maximum blob detection size in the image
		double m_MaxDetectionSize;
		// Maximum force magnitude
		double m_MaxForce;
		// Minimum force magnitude
		double m_MinForce;
	};
}

#endif // ROBOTCONTROL_H