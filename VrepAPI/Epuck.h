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


#ifndef EPUCK_H
#define EPUCK_H

#include "Robot.h"
#include "Vector2.h"
#include "Log.h"
#include "Motor.h"
#include "EpuckProximitySensor.h"
#include "VisionSensor.h"
#include "RobotControl.h"

#ifdef USE_IMAGEPROCESSOR
#include "ImageProcessor.h"
#endif

namespace VrepAPI
{
	// Standard Epuck class which manages components one by one
	class Epuck
		: public Robot
	{
	public:
		Epuck(std::string name);
		Epuck(int handle, std::string name);

		virtual ~Epuck();

		// Load motor from V-REP
		virtual void Load(VrepConnectorPtr& vrepConnectorPtr);

		// Write and read values to V-REP server
		virtual void Update(VrepConnectorPtr& vrepConnectorPtr);
		
		// Retrieve the wheel radius
		double GetWheelRadius() const;

		// Retrieve the robot radius
		double GetRadius() const;
		
	protected:

		Utils::LogPtr m_Logger;

		MotorPtr m_MotorRightPtr;
		MotorPtr m_MotorLeftPtr;

		const double m_WheelRadius;
		const double m_HalfAxleLength;
		const double m_Radius;

		// Maximum force magnitude
		const double m_ForceMaximum;

		// Minimum force magnitude
		const double m_ForceMinimum;

		// These values represent scaling factors for the force
		const double m_DetectionDistanceMax;
		const double m_DetectionDistanceMin;

		double CalculateForcePush(double distance);
		double CalculateForcePull(double distance);

		// Calculates the speed if no reference is given
		Vector2 CalculateSpeed();

		// Transforms the robot speed = (linear, angular) in to wheel speed
		void SetWheelSpeed(Vector2 robotSpeed);

	private:

		// Get proximity sensor data
		Vector2 GetProximitySensorForce();

		std::vector<EpuckProximitySensorPtr> m_ProximitySensors;

		VisionSensorPtr m_VisionSensorPtr;

		RobotControl m_RobotControl;

		#ifdef USE_IMAGEPROCESSOR
		ImageProcessing::ImageProcessorPtr m_ImageProcessorPtr;
		#endif

		// Checks the proximity sensors and discards multiple detections of the same objects
		void ProcessProximitySensors();
	};

	typedef boost::shared_ptr<Epuck> EpuckPtr;
}

#endif // EPUCK_H

// Epuck sensor layout
//							  x axis
//							  ^	 	
//			rot+			  |					rot-
//							front
//						ps3		 ps4
//					 ps2		    ps5
//			left  ps1    	Epuck	   ps6  right  ---> y axis
//					 ps8			ps7
//					        back
//
// Epuck proximity sensor angle [Degree] offset table
// Device	from xEpuck			to xEpuck
// ps1		90.0				-90.0
// ps2		45.85987261			-45.85987261
// ps3		17.19745223			-17.19745223
// ps4		-17.19745223		17.19745223
// ps5		-45.85987261		45.85987261
// ps6		-90.0				90
// ps7		-151.3375796		151.3375796
// ps8		151.3375796			-151.3375796

// In case of each proximity sensor x axis is in front, y axis is right