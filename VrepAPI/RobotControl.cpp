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


#include "RobotControl.h"

namespace VrepAPI
{
	RobotControl::RobotControl()
		: m_Trashold(1.0),
		m_MaxSpeed(10),
		m_MaxForce(100),
		m_MinForce(10),
		m_MinSpeed(.2),
		m_KW(.001275),
		m_ProximitySensorForce(0,0),
		m_IsCloseEnough({ { 0, false }, { 1, false }, { 2, false } }),
		m_MaxDetectionSize(30.0) // [px] -> diameter of the meaningful keypoint neighborhood
	{
		m_SpeedMaximum.x = 0.3; // [m/s]
		m_SpeedMaximum.y = 0.0; // [rad/s]

		m_SpeedMinimum.x = 0.05; // [m/s]
		m_SpeedMinimum.y = 0.0; // [rad/s]

		SetDefaultWeights();
	}

	void RobotControl::SetDetectedPoints(map<int, map<string, double>> points)
	{
		this->m_DetectedPointList = points;

		m_Weights.clear();
		m_Weights.empty();
		GetWeights();
		if (m_Weights.size() == 0) SetDefaultWeights();

		m_MaxBlobSize = .0;
		GetMaxBlobSize();
	}

	void RobotControl::SetProximitySensorForce(Vector2 force)
	{
		this->m_ProximitySensorForce = force;
	}

	Vector2 RobotControl::CalculateSpeed()
	{
		double v = .0;
		double w = .0;
		// The robot is detecting in a [-30..30] degree interval
		// Maximum speed forward
		Vector2 F2(m_MaxForce, .0);  // Unit: degrees
		// Maximum speed to left (-30 degrees)
		Vector2 F1 = Vector2(m_MaxForce, .0).Rotate(-30.0);  // Unit: degrees
		// Maximum speed to right (30 degrees)
		Vector2 F3 = Vector2(m_MaxForce, .0).Rotate(30.0);  // Unit: degrees

		// Speed calculated from the data of received image
		m_CameraFroce = F1 * m_Weights[0] + F2 * m_Weights[1] + F3 * m_Weights[2]; // max: 30

		// Camera force should not exceed limit
		m_CameraFroce.Clamp(m_MaxForce);

		// Final force = k1 * camera force + k2 * proximity force
		m_Force = .6 * m_CameraFroce + .4 * m_ProximitySensorForce;
		
		// The owerall sum should not exceed speed limit
		m_Force.Clamp(m_MaxForce);

		// Calculate linear and angular speed
		double d = m_Force.Magnitude();
		if (d > m_MinSpeed)
		{
			double m = (m_SpeedMaximum.x - m_SpeedMinimum.x) / (m_MaxForce - m_MinForce);
			double n = m_SpeedMinimum.x - m  * m_MinForce;

			v = d * m + n;
			w = m_KW * (int)(.02 + m_Force.Angle());
		}

		return Vector2(v, w);
	}

	void RobotControl::GetWeights()
	{
		double velocity = .0;
		for (auto point : m_DetectedPointList)
		{
			if (m_MaxBlobSize > 0 && point.second["size"] > 0)
				point.second["size"] /= m_MaxBlobSize;
			else
				point.second["size"] = 0;

			auto originalBlobSize = point.second["size"] * m_MaxBlobSize;
			if (originalBlobSize > m_MaxDetectionSize)
				m_IsCloseEnough[point.first] = true;
			else
				m_IsCloseEnough[point.first] = false;
			
			double weight = .5 * point.second["size"];
			m_Weights.insert(make_pair(point.first, weight));
		}
	}

	void RobotControl::GetMaxBlobSize()
	{
		double maxBlobSize = 0;
		for (auto size : m_DetectedPointList)
		{
			if (size.second["size"] > maxBlobSize) maxBlobSize = size.second["size"];
		}

		m_MaxBlobSize = maxBlobSize;
	}

	void RobotControl::SetDefaultWeights()
	{
		m_Weights.insert(make_pair(0, .0));
		m_Weights.insert(make_pair(1, .0));
		m_Weights.insert(make_pair(2, .0));
	}
}