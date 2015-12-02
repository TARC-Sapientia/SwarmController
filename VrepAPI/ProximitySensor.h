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


#ifndef PROXIMITYSENSOR_H
#define PROXIMITYSENSOR_H

#include "VrepSimpleObject.h"

namespace VrepAPI
{
	// A general VREP proximity sensor prototype with direct calls to simulator
	// If multiple obstacles are in range only the closest object is considered
	class ProximitySensor : 
		public VrepSimpleObject
	{
	public:
		ProximitySensor(int handle, std::string name);
		ProximitySensor(std::string name);

		virtual ~ProximitySensor();

		// Load the object properties
		virtual void Load(VrepConnectorPtr& vrepConnectorPtr);

		// Write and/or read data from V-REP
		virtual void Update(VrepConnectorPtr& vrepConnectorPtr);

		// The other data is valid only if this function returns true
		virtual bool IsDetecting();

		// Get the detected robot/object's handle 
		virtual int GetDetectedObjectHandle();

		// Get the detected point's x,y,z coordinates
		virtual std::vector<double> GetDetectedPoint();

		// Get the detected surface's normal vector nx,ny,nz coordinates
		virtual std::vector<double> GetDetectedNormalVector();

	private:
		
		// Detection state. If false all other values should be ignored
		bool m_DetectionsState;

		// The x,y,z position of the closest detected point relative to the sensor reference frame
		std::vector<double> m_DetectedPoint;

		// The handle of the detected object 
		int m_DetectedObjectHandle;

		// The normal vector (normalized) of the detected surface. Relative to the sensor reference frame
		std::vector<double> m_DetectedSurfaceNormalVector;
	};

	typedef boost::shared_ptr<ProximitySensor> ProximitySensorPtr;
}

#endif // PROXIMITYSENSOR_H