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


#ifndef OBSTACLE_H
#define OBSTACLE_H

#include "VrepSimpleObject.h"

namespace VrepAPI
{
	class Obstacle :
		public VrepSimpleObject
	{
	public:
		Obstacle(int handle, std::string name);
		Obstacle(std::string name);

		virtual ~Obstacle();

		// Load motor from V-REP
		virtual void Load(VrepConnectorPtr& vrepConnectorPtr);

		// Write and read values to V-REP server.
		virtual void Update(VrepConnectorPtr& vrepConnectorPtr);
	};

	typedef boost::shared_ptr<Obstacle> ObstaclePtr;
}

#endif // OBSTACLE_H