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


#ifndef VREPSIMPLEOBJECT_H
#define VREPSIMPLEOBJECT_H

#include "VrepObject.h"
#include "VrepConnector.h"
#include <boost\weak_ptr.hpp>

namespace VrepAPI
{
	// Represents a leaf object in VREP hierarchy with no siblings
	class VrepSimpleObject
		: public VrepObject
	{
	public:
		VrepSimpleObject(std::string name);
		VrepSimpleObject(int handle, std::string name);

		virtual ~VrepSimpleObject();

		// Use only a weak pointer to prevent circular referencing
		VrepObjectWeakPtr GetParent() const;

		void SetParent(VrepObjectPtr& parent);

		// Load the object properties
		virtual void Load(boost::shared_ptr<VrepConnector>& vrepConnectorPtr) = 0;

		// Write and/or read data from V-REP
		virtual void Update(boost::shared_ptr<VrepConnector>& vrepConnectorPtr) = 0;

	private:

		VrepObjectPtr m_Parent;
	};
}

#endif // VREPSIMPLEOBJECT_H