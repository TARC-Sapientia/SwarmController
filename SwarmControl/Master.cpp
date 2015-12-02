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


#include "Master.h"
#include "Utilities.h"
#include "Epuck.h"
#include "YouBot.h"

using namespace std;
using namespace std::chrono;
using namespace Haptics;
using namespace Utils;
using namespace SwarmControl;
using namespace VrepAPI;
//using namespace InputReader;

const std::string scenePath(""); // does not load scene
//const std::string scenePath("D:\\svn\\TO\\Simulations\\SwarmControl\\Scenes\\EpuckSmallScene.ttt");

Master::Master()
	: m_HapticManagerPtr(new HapticManager("")),
	m_ControlTimer(Core::get().getIoService()),
	m_ControlTimerPeriod(250),
	m_VrepConnectorPtr(new VrepConnector()),
	m_Logger(new Log("Log", "Master")),
	m_LoggerSwarm(new Log("Log", "Swarm")),
	m_IsConnectedHapticP1(false),
	m_IsProximitySensorGroupDataInitialized(false),
	m_IsSimulationRunning(false),
	m_IsThreadedRenderingEnabled(true),
	m_SpeedReference(0.10, 0.0)
{
}

Master::~Master()
{
	Stop(); 

	m_VrepConnectorPtr->ConnectionStop();
	
	m_Robots.clear();
	m_Obstacles.clear();
}

void Master::Initialize()
{
	// Check haptic device status
	m_IsConnectedHapticP1 = m_HapticManagerPtr->IsConnected(Haptics::PHANToM_H1);

	// Connect to VREP 
	m_VrepConnectorPtr->ConnectionStart("127.0.0.1", 19997);

	// Load scene if necesary 
	if (scenePath.size() > 0) m_VrepConnectorPtr->LoadScene(scenePath);

	// Retreive all objects from V-REP connector
	auto objects = m_VrepConnectorPtr->GetAllObjects();

	// Fill corresponding list accordingly 
	for (auto& object : objects)
	{
		if (auto robot = boost::dynamic_pointer_cast<Robot>(object.second))
		{
			// Add robot to robot map
			m_Robots.insert({ object.first, robot });

			auto name = robot->GetName();
		}

		if (auto obstacle = boost::dynamic_pointer_cast<Obstacle>(object.second))
		{
			m_Obstacles.insert({ object.first, obstacle });
		}
	}
	
	// Load robots 
	for (auto& robot : m_Robots)
	{
		robot.second->Load(m_VrepConnectorPtr);
	}

	stringstream ss;
	ss << "Controltimer period[ms]: " << m_ControlTimerPeriod << endl
		<< "Speed reference [m/s, rad/s]: " << m_SpeedReference.ToString() << endl
		<< "Stop when first robot reaches [m]: 10.0" << endl
		<< "Total robot count: " << m_Robots.size() << endl;

	m_Logger->Write(ss.str());
}

void Master::Start()
{
	m_VrepConnectorPtr->SimulationStart();

	if (m_IsThreadedRenderingEnabled == true)
	{
		m_VrepConnectorPtr->SetThreadedRendering(true);
	}
	
	StartControlTimer();

	m_IsSimulationRunning = true;

	m_SimulationStartTime = high_resolution_clock::now();

	std::cout << "Start simulation..." << std::endl;
}

void Master::Stop()
{
	if (m_IsSimulationRunning == true)
	{
		StopControlTimer();

		m_VrepConnectorPtr->SimulationStop();
		
		m_IsSimulationRunning = false;

		auto endTime = high_resolution_clock::now();
		auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - m_SimulationStartTime).count();
		m_Logger->Write("Simulation time[ms]: " + to_string(duration) + "\n");

		std::cout << "Stop simulation..." << std::endl;
	}
}

void Master::StartControlTimer()
{
	m_ControlTimer.expires_from_now(boost::posix_time::milliseconds(m_ControlTimerPeriod));

	m_ControlTimer.async_wait(boost::bind(&Master::OnControlTimerElapsed, this, boost::asio::placeholders::error));
}

void Master::StopControlTimer()
{
	m_ControlTimer.cancel();
}

void Master::OnControlTimerElapsed(const boost::system::error_code& error_code)
{
	if (error_code)
	{
		// Timer has been canceled - or some error occurred
		return;
	}

	StartControlTimer();
	
	GetHapticDeviceData();

	m_VrepConnectorPtr->UpdateAllProximitySensorData(m_IsProximitySensorGroupDataInitialized);

	Vector2 swarmVelocity;
	Vector2 swarmPosition;
	double swarmOrientation = 0;

	for (auto& robot : m_Robots)
	{
		if (auto epuck = boost::dynamic_pointer_cast<Epuck>(robot.second))
		{
			// Add the velocity
			swarmVelocity += epuck->GetLinearVelocity();

			// Add the position: only x,y are considered
			auto position = epuck->GetPosition();
			swarmPosition += Vector2(position[0], position[1]);

			// Add orientation: only beta is considered
			swarmOrientation += epuck->GetOrientation()[1];

			if (abs(10.0 + position[1]) < 0.01)
			{
				Stop();
				cout << "Distance limit reached." << endl;
				return;
			}
		}

		if (auto youbot = boost::dynamic_pointer_cast<YouBot>(robot.second))
		{
			youbot->SetWheelSpeed(
					m_HapticPositions.z,						//v
					m_HapticPositions.gimbalAngle,			//w
					m_HapticPositions.x						//wz
				);
		}

		// Update the robot
		robot.second->Update(m_VrepConnectorPtr);
	}

	if (m_IsProximitySensorGroupDataInitialized == false)
	{
		m_IsProximitySensorGroupDataInitialized = true;
	}
	else
	{
		swarmVelocity /= m_Robots.size();
		swarmPosition /= m_Robots.size();
		swarmOrientation /= m_Robots.size();

		// Log status
		m_LoggerSwarm->Write(
			"x: " + to_string(swarmPosition.x) + "\ty: " + to_string(swarmPosition.y)
			+ "\tspeedLinearRef: " + to_string(m_SpeedReference.x)
			+ "\tspeedLinear: " + to_string(swarmVelocity.Magnitude())
			+ "\tspeedAngularRef: " + to_string(m_SpeedReference.y)
			+ "\tspeedAngular: " + to_string(swarmOrientation)
			+ "\n");
	}
}

void Master::GetHapticDeviceData()
{
	m_HapticPositions.x = 0;
	m_HapticPositions.y = 0;
	m_HapticPositions.z = 0;
	m_HapticPositions.gimbalAngle = 0;

	// Select prefered haptic device
	if (m_IsConnectedHapticP1)
	{
		m_HapticData = m_HapticManagerPtr->GetData(Haptics::PHANToM_H1);
	

		if (m_HapticManagerPtr->IsDarkGreyButtonPressed(Haptics::PHANToM_H1))
		{
			// X, Y, Z positions from Haptic Device
			m_HapticPositions.x = m_HapticData.position[0];
			m_HapticPositions.y = m_HapticData.position[1];
			m_HapticPositions.z = m_HapticData.position[2];
			m_HapticPositions.gimbalAngle = m_HapticData.gimbalAngles[0];
		}
	}

}