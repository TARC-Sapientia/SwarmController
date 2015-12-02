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


#ifndef VREPCONNECTOR_H
#define VREPCONNECTOR_H

#include "VrepBasicConnector.h"
#include "VrepObject.h"
#include <map>

namespace VrepAPI
{
	class Vector2;

	// Holds all objects from V-REP simulator and provides advanced functionality
	class VrepConnector
		: public VrepBasicConnector
	{
	public:

		VrepConnector();
		~VrepConnector();

		// Update all proximity sensor data at once. Instead of each sensor requesting the data. 
		// This function reads the all the proximity sensor data from V-REP and updates each sensor with this data
		void UpdateAllProximitySensorData(bool isStreamingInitialized = true);

		// Retrieve a raw object pointer based on it's name
		VrepObjectPtr& GetRawVrepObject(std::string name);

		// Get the name based on the stored objects found at the beginning
		std::string GetObjectName(int handle);

		// Look for the name directly
		std::string GetObjectNameDirectly(int handle);

		// Read the VREP object hierarchy. Executed only once after connecting to the V-REP server
		std::map<std::string, VrepObjectPtr> GetAllObjects();

		// TODO: not working
		// Read the positions in global coordinates of all objects found in the scene
		// In case complex objects the position of each component is retrieved
		// This adds an extra time penalty because of the unnecessary positions 
		std::map<std::string, Vector2> GetAllObjectPositions(bool isStreamingInitialized = true);

	private:

		// Store all object found in VREP in <vrep_name, object_ptr> pair
		std::map<std::string, VrepObjectPtr> m_Objects;

		// Fast access: <handle, name>
		std::map<int, std::string> m_NameMap;

		// Fast access: <handle, topParentHandle>
		std::map<int, int> m_TopParentMap;
	};

	typedef boost::shared_ptr<VrepConnector> VrepConnectorPtr;
}

#endif // VREPCONNECTOR_H