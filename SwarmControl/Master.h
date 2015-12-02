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


#ifndef MASTER_H
#define MASTER_H

#include <vector>
#include <boost\shared_ptr.hpp>
#include <chrono>
#include <conio.h>
#include "Core.h"
#include "Log.h"
#include "HapticManager.h"
#include "VrepConnector.h"
#include "Robot.h"
#include "Obstacle.h"
#include "Vector2.h"
//#include "KeyboardReader.h"

namespace SwarmControl
{
	class Master
	{
	public:
		Master();
		~Master();

		void Start();
		void Stop();
		void Initialize();

	private:
		
		struct {
			double x;
			double y;
			double z;
			double gimbalAngle;
		} m_HapticPositions;

		Haptics::HapticManagerPtr m_HapticManagerPtr;
		
		bool m_IsConnectedHapticP1;

		VrepAPI::VrepConnectorPtr m_VrepConnectorPtr;

		// The robot map: (name of the robot, robotPtr)
		std::map<std::string, VrepAPI::RobotPtr> m_Robots;
		
		// The obstacle map: (name of the obstacle, obstaclePtr)
		std::map<std::string, VrepAPI::ObstaclePtr> m_Obstacles;

		// The reference speed x = linear[m/s], y = angular[rad/s])
		VrepAPI::Vector2 m_SpeedReference;

		int m_ControlTimerPeriod;

		void OnControlTimerElapsed(const boost::system::error_code& error_code);
		boost::asio::deadline_timer m_ControlTimer;

		void StartControlTimer();
		void StopControlTimer();

		bool m_IsSimulationRunning;

		// Set up proximity sensor data group streaming
		bool m_IsProximitySensorGroupDataInitialized;
		
		// Rendering run on a separat thread
		const bool m_IsThreadedRenderingEnabled;

		// Log master related data 
		Utils::LogPtr m_Logger;

		// Log swarm related data
		Utils::LogPtr m_LoggerSwarm;

		// Data received from haptic device
		Haptics::HapticData m_HapticData;

		// Read data from haptic device
		void GetHapticDeviceData();

		// KeyboardReader
		//InputReader::KeyboardReader m_KeyboardReader;

		// Keyboard data
		//map<string, int> m_KeyboardData;

		// Read data from keyboard
		//void ReadKeyboardInput();

		std::chrono::high_resolution_clock::time_point m_SimulationStartTime;
	};

	typedef boost::shared_ptr<Master> MasterPtr;
}

#endif //MASTER_H