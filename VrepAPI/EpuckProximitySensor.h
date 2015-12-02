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


#ifndef EPUCKPROXIMITYSENSOR_H
#define EPUCKPROXIMITYSENSOR_H

#include "VrepSimpleObject.h"
#include "Vector2.h"
#include <vector>

namespace VrepAPI
{
	// Fast proximity sensor for epuck robot with no direct connection to VREP. 
	// Stores the sensor specific data from the group data call.
	// The stored values are relative to the robot which contains the sensor.
	// In case of multiple objects only the closest is considered.
	class EpuckProximitySensor :
		public VrepSimpleObject
	{
	public:
		EpuckProximitySensor(std::string name);
		EpuckProximitySensor(int handle, std::string name);

		~EpuckProximitySensor();

		// Load the object properties. Not used
		void Load(VrepConnectorPtr& vrepConnectorPtr) { }

		// Write and/or read data from V-REP. Not used
		void Update(VrepConnectorPtr& vrepConnectorPtr) { }
		
		// Store the data when detecting
		// The point is in the sensor's reference frame
		void SetData(const VrepObjectWeakPtr& rawObject, std::vector<double> point);

		// Update the detection status
		void SetDetectionStatus(bool isDetecting);

		// The other data is valid only if this function returns true
		bool IsDetecting() const;

		// The detected point in the robot's reference frame
		Vector2 GetPoint() const;

		// The detected distance [m] from the robot's center and the detected point
		double GetDistance() const;

		// The angle [degrees: -180 : 180] between the robot's heading and the detected point
		double GetAngle() const;

		// A weak pointer to the detected object
		const VrepObjectWeakPtr& GetObjectWeakptr() const;
		
	private:

		// The detection is only valid if this is true
		// The data remains valid until this flag is reseted
		bool m_IsDetecting;

		// The detected object
		VrepObjectWeakPtr m_DetectedObject;

		// The detected point in the robot's reference frame
		Vector2 m_DetectedPoint;

		// The distance [m] from the detected robot/object's relative to the sensor reference frame
		double m_DetectedDistance;

		// The detected angle [degrees] from the robot's heading angle
		double m_DetectedAngle;

		// The distance offset from the robot's center
		double m_DistanceOffset;

		// The angle offset relative to the robot's heading. Const
		double m_AngleOffset;

		// The sin of the angle offset. Const
		double m_AngleOffsetSin;

		// The cos of the angle offset. Const
		double m_AngleOffsetCos;
	};

	typedef boost::shared_ptr<EpuckProximitySensor> EpuckProximitySensorPtr;
}
#endif // EPUCKPROXIMITYSENSOR_H