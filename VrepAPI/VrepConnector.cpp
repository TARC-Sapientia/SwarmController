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


#include "VrepConnector.h"
#include "Motor.h"
#include "Epuck.h"
#include "EpuckProximitySensor.h"
#include "VisionSensor.h"
#include "YouBot.h"
#include "Obstacle.h"
#include "VrepGeneralObject.h"
#include "Vector2.h"

extern "C"
{
#include "extApi.h"
//#include "extApiCustom.h" // Custom commands 
#include "extApiPlatform.h"
}

#include <iostream>
#include <vector>
#include <chrono>

namespace VrepAPI
{
	VrepConnector::VrepConnector()
	{
	}

	VrepConnector::~VrepConnector()
	{
		m_Objects.clear();
	}

	std::map<std::string, VrepObjectPtr> VrepConnector::GetAllObjects()
	{
		int objectType = sim_appobj_object_type; // all objects
		int dataType = 0;
		std::vector<int> handles;
		std::vector<int> ints;
		std::vector<double> doubles;
		std::vector<std::string> strings;

		// Get object names 
		dataType = 0; // 0: retrieves the object names (in stringData.)
		GetObjectGroupData(objectType, dataType, handles, ints, doubles, strings);

		// Save object names
		auto objectNames = strings;

		dataType = 1; // 1: retrieves the object types (in intData)
		GetObjectGroupData(objectType, dataType, handles, ints, doubles, strings);

		// Save object handles and types
		auto objectHandles = handles;
		auto objectTypes = ints;

		dataType = 2; // 2 : retrieves the parent object handles(in intData)
		GetObjectGroupData(objectType, dataType, handles, ints, doubles, strings);

		// Save object parent handles
		auto objectParentHandles = ints;

		for (unsigned int i = 0; i < handles.size(); i++)
		{
			// Store variables for fast access
			m_NameMap.insert({ objectHandles[i], objectNames[i] });

			if (objectNames[i].find("Wall") != std::string::npos ||
				objectNames[i].find("Rectangle") != std::string::npos)
			{
				m_Objects.insert({ objectNames[i], VrepObjectPtr(new Obstacle(objectHandles[i], objectNames[i])) });
			}
			else
			{
				if (objectTypes[i] == sim_object_joint_type)
				{
					m_Objects.insert({ objectNames[i], VrepObjectPtr(new Motor(objectHandles[i], objectNames[i])) });
				}
				else if (objectTypes[i] == sim_object_proximitysensor_type)
				{
					m_Objects.insert({ objectNames[i], VrepObjectPtr(new EpuckProximitySensor(objectHandles[i], objectNames[i])) });
				}
				else if (objectTypes[i] == sim_object_visionsensor_type)
				{
					m_Objects.insert({ objectNames[i], VrepObjectPtr(new VisionSensor(objectHandles[i], objectNames[i])) });
				}
				else if (objectParentHandles[i] == -1 && objectNames[i].find("ePuck") != std::string::npos)
				{
					m_Objects.insert({ objectNames[i], VrepObjectPtr(new Epuck(objectHandles[i], objectNames[i])) });
				}
				else if (objectParentHandles[i] == -1 && objectNames[i].find("youBot") != std::string::npos)
				{
					m_Objects.insert({ objectNames[i], VrepObjectPtr(new YouBot(objectHandles[i], objectNames[i])) });
				}
				else
				{
					m_Objects.insert({ objectNames[i], VrepObjectPtr(new VrepGeneralObject(objectHandles[i], objectNames[i])) });
				}
			}
		}

		// Fix parents and children
		for (auto& object : m_Objects)
		{
			if (auto simpleObject = boost::dynamic_pointer_cast<VrepSimpleObject>(object.second))
			{
				// Locate top parent handle
				auto handle = simpleObject->GetHandle();

				while (objectParentHandles[handle] != -1) // -1 means no parent
				{
					handle = objectParentHandles[handle];
				}

				// Store variables for fast access
				m_TopParentMap.insert({ object.second->GetHandle(), handle });

				// Set the parent
				auto parentName = objectNames[handle];
				simpleObject->SetParent(m_Objects[parentName]);

				if (auto complexObject = boost::dynamic_pointer_cast<VrepComplexObject>(m_Objects[parentName]))
				{
					// Add this child to the parent
					complexObject->AddComponent(object.second);
				}
			}
		}

		return m_Objects;
	}

	VrepObjectPtr& VrepConnector::GetRawVrepObject(std::string name)
	{
		if (m_Objects.count(name) == 1)
		{
			return m_Objects[name];
		}

		// Key is not found
		throw new std::string("Unable to locate object: " + name);
	}

	std::string VrepConnector::GetObjectName(int handle)
	{
		for (const auto& object : m_Objects)
		{
			if (object.second->GetHandle() == handle)
			{
				return object.second->GetName();
			}
		}

		throw new std::string("Unable to locate object by name");
	}

	std::string VrepConnector::GetObjectNameDirectly(int handle)
	{
		int objectType = sim_appobj_object_type; // all objects
		int dataType = 0; // 0: retrieves the object names (in stringData.)
		std::vector<int> handles;
		std::vector<int> ints;
		std::vector<double> doubles;
		std::vector<std::string> strings;

		// Get object names 
		dataType = 0; // 0: retrieves the object names (in stringData.)
		GetObjectGroupData(objectType, dataType, handles, ints, doubles, strings);

		for (auto & h : handles)
		{
			if (handle == h)
			{
				return strings[h];
			}
		}

		throw new std::string("Unable to locate object by name");
	}

	void VrepConnector::UpdateAllProximitySensorData(bool isStreamingInitialized)
	{
		int objectType = sim_object_proximitysensor_type;

		// 13: retrieves proximity sensor data (in intData (2 values): detection state, detected object handle. 
		// In floatData (6 values): detected point (x,y,z) and detected surface normal (nx,ny,nz))
		int dataType = 13;
		std::vector<int> handles;
		std::vector<int> ints;
		std::vector<double> doubles;
		std::vector<std::string> strings;

		GetObjectGroupData(objectType, dataType, handles, ints, doubles, strings, isStreamingInitialized);

		for (unsigned int i = 0; i < handles.size(); i++)
		{
			if (ints[i * 2] == 1) // Detection state of the current sensor
			{
				auto detectedObjectHanlde = ints[i * 2 + 1];
				auto detectedObject = m_Objects[m_NameMap[m_TopParentMap[detectedObjectHanlde]]];

				auto detectedPoint = { doubles[i * 6 + 0], doubles[i * 6 + 1], doubles[i * 6 + 2] };
				
				// Assign data to the sensor
				auto proximitySensorName = m_NameMap[handles[i]];
				auto proximitySensor = boost::dynamic_pointer_cast<EpuckProximitySensor>(m_Objects[proximitySensorName]);				
				proximitySensor->SetData(detectedObject, detectedPoint);
			}
		}
	}

	std::map<std::string, Vector2> VrepConnector::GetAllObjectPositions(bool isStreamingInitialized)
	{
		std::map<std::string, Vector2> positionMap;
		int objectType = sim_appobj_object_type 
			& !sim_object_proximitysensor_type
			& !sim_object_joint_type
			& !sim_object_graph_type
			& !sim_object_camera_type
			& !sim_object_dummy_type
			& !sim_object_proximitysensor_type
			& !sim_object_reserved1
			& !sim_object_reserved2
			& !sim_object_path_type
			& !sim_object_visionsensor_type
			& !sim_object_volume_type
			& !sim_object_mill_type
			& !sim_object_forcesensor_type
			& !sim_object_light_type
			& !sim_object_mirror_type
			& !sim_appobj_collision_type
			& !sim_appobj_distance_type
			& !sim_appobj_simulation_type
			& !sim_appobj_ik_type
			& !sim_appobj_constraintsolver_type
			& !sim_appobj_collection_type
			& !sim_appobj_ui_type
			& !sim_appobj_script_type
			& !sim_appobj_pathplanning_type
			& !sim_appobj_RESERVED_type
			& !sim_appobj_texture_type
			& !sim_appobj_motionplanning_type
			;

		// 3: retrieves the absolute object positions(in floatData.There are 3 values for each object(x, y, z))
		int dataType = 3;

		std::vector<int> handles;
		std::vector<int> ints;
		std::vector<double> doubles;
		std::vector<std::string> strings;

		GetObjectGroupData(objectType, dataType, handles, ints, doubles, strings, isStreamingInitialized);

		for (unsigned int i = 0; i < handles.size(); i++)
		{
			std::string name = m_NameMap[handles[i]];

			if (name.find("ePuck") != std::string::npos
				&& name.find("_") == std::string::npos)	// not a component
			{
				// Epuck only 
				//cout << name << '\t'				// global coordinates
				//	<< doubles[i * 3 + 0] << '\t'	// x axis 
				//	<< doubles[i * 3 + 1] << '\t'	// y axis
				//	//<< doubles[i * 3 + 2];	// z always constant 
				//	<< endl;

				positionMap.insert({ name, Vector2(doubles[i * 3 + 0], doubles[i * 3 + 1]) });
			}
		}

		return positionMap;
	}
}