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


#ifndef HAPTICMANAGER_H
#define HAPTICMANAGER_H

#include "Haptic.h"
#include <string>
#include <boost/shared_ptr.hpp>

namespace Haptics
{
	class HapticManager
	{
	public:
		HapticManager(std::string configFileName);
		~HapticManager();

		HapticData GetData(HapticDevice device);
		void SetForce(HapticDevice device, double force[3]);

		bool IsConnected(HapticDevice device);

		bool IsDarkGreyButtonPressed(HapticDevice device);
		bool IsLightGreyButtonPressed(HapticDevice device);
	};

	typedef boost::shared_ptr<HapticManager> HapticManagerPtr;

} // end namespace

#endif // HAPTICMANAGER_H
