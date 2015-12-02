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


#ifndef VREPOBJECT_H
#define VREPOBJECT_H

#include <boost/shared_ptr.hpp>
#include <string>
#include <vector>

namespace VrepAPI
{
	class VrepConnector;

	// V-REP object prototype. Any object found in the V-REP hierarchy is represented by this class
	class VrepObject
	{
	public:

		VrepObject(std::string name);

		VrepObject(int handle, std::string name);

		VrepObject(const VrepObject& other);

		// Using virtual destructor for proper cleanup of derived classes
		virtual ~VrepObject();

		// Get the V-REP object handle
		int GetHandle() const;

		// Get the V-REP object name
		std::string GetName() const;

		// Get the position in V-REP in global reference frame: x, y, z
		std::vector<double> GetPosition() const;
		
		// Get the orientation in V-REP in global reference frame, Euler angles: alpha, beta, gamma
		std::vector<double> GetOrientation() const;

		// Get the status of the data streaming
		bool IsStreamingInitialized() const;
		
		// Load the object properties
		virtual void Load(boost::shared_ptr<VrepConnector>& vrepConnectorPtr) = 0;

		// Write and/or read data from V-REP
		virtual void Update(boost::shared_ptr<VrepConnector>& vrepConnectorPtr) = 0;

	protected:

		// The V-REP object handle
		int m_Handle;

		// The V-REP object identifier name
		std::string m_Name;

		// The position in V-REP: x, y, z. In global reference frame.
		std::vector<double> m_Position;

		// The orientation in V-REP, the Euler angles: alpha, beta, gamma. In global reference frame.
		std::vector<double> m_Orientation;

		// Used to initialize data streaming
		// If false the streamming command will be used
		// If true the buffer command will be used
		bool m_IsStreamingInitialized;
	};

	typedef boost::shared_ptr<VrepObject> VrepObjectPtr;
	typedef boost::weak_ptr<VrepObject> VrepObjectWeakPtr;
}

#endif // VREPOBJECT_H
