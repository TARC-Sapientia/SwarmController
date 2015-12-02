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


#ifndef VREPBASICCONNECTOR_H
#define VREPBASICCONNECTOR_H

#include <vector>

namespace VrepAPI
{
	// Provides only basic direct API calls to V-REP
	class VrepBasicConnector
	{
	public:

		VrepBasicConnector();
		virtual ~VrepBasicConnector();

		static const int CONNECTION_TIMEOUT = 2000;
		static const int CONNECTION_CYCLE = 1;

		void Init();
		void ConnectionStart(const std::string ip, int port);
		void ConnectionStop();

		void CommunicationPause(bool mode);

		void SimulationStart();
		void SimulationStop();
		void SimulationPause();

		void LoadScene(std::string scenePath);

		// If enabled the rendering will be done on a dedicated thread. 
		// When the threaded rendering mode is activated, a simulation cycle will only consist in execution of the main script,
		// thus simulations will run at maximum speed.Rendering will happen via a different thread, and not slow down the simulation task.
		// The drawbacks have however to be considered.When threaded rendering is activated, then:
		// - Rendering will happen asynchronously to the simulation loop, and visual glitches might appear
		// - The video recorder will not operate at constant speed(some frames might get skipped)
		// - The stability of the application might be reduced
		// - Some operations(e.g.erasing an object, etc.) require to wait for the rendering thread to finish work,
		//   before being able to execute, and vice - versa.In those situations, cycles could take more time than in the sequential rendering mode.
		void SetThreadedRendering(bool enabled);

		// Read an object handle from V-REP based on it's name
		int GetObjectHandle(std::string name);

		// Read the position of an object from V-REP
		// Specify -1 to absolute orientation. Otherwise an object handle relative to whose reference frame you want the position
		std::vector<double> GetObjectPosition(int handle, int relativeToObjectHandle, bool isStreamingInitialized = true) const;

		// Read the orientation of an object from V-REP
		// Specify -1 to absolute orientation. Otherwise an object handle relative to whose reference frame you want the orientation
		std::vector<double> GetObjectOrientation(int handle, int relativeToObjectHandle, bool isStreamingInitialized = true) const;

		// Using Streaming and Buffer depending isStreamingInitialized
		void GetObjectGroupData(int objectType, int dataType,
			std::vector<int>& handles, std::vector<int>& ints, std::vector<double>& doubles, std::vector<std::string>& strings,
			bool isStreamingInitialized) const;

		// Using one shot wait
		void GetObjectGroupData(int objectType, int dataType,
			std::vector<int>& handles, std::vector<int>& ints, std::vector<double>& doubles, std::vector<std::string>& strings) const;

		// Return true if dynamic is enabled for the given joint
		bool GetMotorDynamic(int handle) const;

		// Return true if position control is enabled for the given joint
		bool GetMotorPositionControl(int handle) const;

		// Send a target position to the given joint
		void SetMotorPosition(int handle, double pos) const;

		// TODO:
		void SetMotorVelocity(int handle, double velocity) const;

		// Send the maximum force torque to apply to a joint
		void SetMotorTorque(int handle, double force) const;

		// Read current joint position
		double GetMotorPosition(int handle, bool isStreamingInitialized = true) const;

		// Read current joint torque
		double GetMotorTorque(int handle, bool isStreamingInitialized = true) const;

		// Read the velocity of an object from V-REP
		void GetObjectVelocity(int handle, std::vector<double>& linearVelocity, std::vector<double>& angularVelocty, bool isStreamingInitialized = true) const;

		// Read the proximity sensor data. If the detection state is false the rest of the values are irrelevant
		void GetProximitySensorData(int handle, bool& detectionState, std::vector<double>& detectedPoint, int& detectedObjectHandle, std::vector<double>& detectedSurfaceNormalVector, bool isStreamingInitialized = true);

		// Read the vision sensor data
		void GetVisionSensorData(int handle, int& height, int& width, std::vector<unsigned char>& imageVector, char options, bool isStreamingInitialized);

	private:

		int m_ClientID;
	};
}

#endif // VREPBASICCONNECTOR_H