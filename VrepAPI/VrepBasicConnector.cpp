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


#include "VrepBasicConnector.h"

extern "C"
{
	#include "extApi.h"
	//#include "extApiCustom.h" // Custom commands 
	#include "extApiPlatform.h"
}

using namespace VrepAPI;

VrepBasicConnector::VrepBasicConnector()
	: m_ClientID(-1)
{
}

VrepBasicConnector::~VrepBasicConnector()
{
	simxFinish(-1);
}

void VrepBasicConnector::ConnectionStart(const std::string ip, int port)
{
	simxFinish(-1);

	// Attempt connection
	m_ClientID = simxStart((simxChar*)ip.c_str(),
		(simxInt)port,
		true, // waitUntilConnected: if different from zero, then the function blocks until connected (or timed out)
		true, // doNotReconnectOnceDisconnected: if different from zero, then the communication thread will not attempt a second connection if a connection was lost.
		CONNECTION_TIMEOUT,
		CONNECTION_CYCLE);

	if (m_ClientID == -1)
	{
		throw std::string("Unable to connect to V-REP Server");
	}
}

void VrepBasicConnector::ConnectionStop()
{
	simxFinish(m_ClientID);
}

void VrepBasicConnector::CommunicationPause(bool mode)
{
	simxInt error = simxPauseCommunication(m_ClientID, mode == false ? (char)0 : (char)1);

	if (error != simx_return_ok)
	{
		throw std::string("Unable to pause the communication");
	}
}

void VrepBasicConnector::SimulationStart()
{
	simxInt error = simxStartSimulation(m_ClientID, simx_opmode_oneshot_wait);

	if (error != simx_error_noerror && error != simx_error_novalue_flag)
	{
		throw std::string("Unable to start the simulation");
	}
}

void VrepBasicConnector::SimulationStop()
{
	simxInt error = simxStopSimulation(m_ClientID, simx_opmode_oneshot_wait);

	if (error != simx_error_noerror && error != simx_error_novalue_flag)
	{
		throw std::string("Unable to stop the simulation");
	}
}

void VrepBasicConnector::SimulationPause()
{
	simxInt error = simxPauseSimulation(m_ClientID, simx_opmode_oneshot_wait);

	if (error != simx_return_ok)
	{
		throw std::string("Unable to pause the simulation");
	}
}

void VrepBasicConnector::LoadScene(std::string scenePath)
{
	simxInt error = simxLoadScene(m_ClientID, scenePath.c_str(), 0, simx_opmode_oneshot_wait);

	if (error != simx_return_ok && error != simx_error_novalue_flag)
	{
		throw std::string("Unable to load scene: " + scenePath);
	}
}

void VrepBasicConnector::SetThreadedRendering(bool enabled)
{
	unsigned char value = enabled == true ? 1 : 0;

	simxInt error = simxSetBooleanParameter(m_ClientID, sim_boolparam_threaded_rendering_enabled, value, simx_opmode_oneshot_wait);

	if (error != simx_return_ok && error != simx_error_novalue_flag)
	{
		throw std::string("Unable to set threaded rendering mode");
	}
}

int VrepBasicConnector::GetObjectHandle(std::string name)
{
	int handle = -1;

	int error = simxGetObjectHandle(m_ClientID, name.c_str(), &handle, simx_opmode_oneshot_wait);
	if (error != simx_return_ok)
	{
		throw std::string("Unable to fetch object handle from name.");
	}

	return handle;
}

bool VrepBasicConnector::GetMotorDynamic(simxInt handle) const
{
	simxInt value;
	simxInt error = 0;

	error = simxGetObjectIntParameter(m_ClientID, handle, 2000, &value, simx_opmode_oneshot_wait);

	if (error == simx_return_ok)
	{
		return value != 0 ? true : false;
	}
	else
	{
		throw std::string("Unable to fetch joint dynamic.");
	}
}

bool VrepBasicConnector::GetMotorPositionControl(simxInt handle) const
{
	simxInt value;
	simxInt error = 0;

	error = simxGetObjectIntParameter(m_ClientID, handle, 2001, &value, simx_opmode_oneshot_wait);

	if (error == simx_return_ok)
	{
		return value != 0 ? true : false;
	}
	else
	{
		throw std::string("Unable to fetch joint position control.");
	}
}

void VrepBasicConnector::SetMotorPosition(int handle, double pos) const
{
	simxInt error = simxSetJointTargetPosition(m_ClientID, handle, (float)pos, simx_opmode_oneshot);

	if (error != simx_return_ok && error != simx_error_novalue_flag)
	{
		throw std::string("Unable to send joint target position.");
	}
}

void VrepBasicConnector::SetMotorVelocity(int handle, double velocity) const
{
	simxInt error = simxSetJointTargetVelocity(m_ClientID, handle, (float)velocity, simx_opmode_oneshot);

	if (error != simx_return_ok && error != simx_error_novalue_flag)
	{
		throw std::string("Unable to set joint target velocity.");
	}
}

void VrepBasicConnector::SetMotorTorque(int handle, double force) const
{
	simxInt error = simxSetJointForce(m_ClientID, handle, (float)force, simx_opmode_oneshot);

	if (error != simx_return_ok && error != simx_error_novalue_flag)
	{
		throw std::string("Unable to send joint maximum torque.");
	}
}

double VrepBasicConnector::GetMotorPosition(int handle, bool isStreamingInitialized) const
{
	simxFloat pos;
	simxInt error = simxGetJointPosition(m_ClientID, handle, &pos, isStreamingInitialized == false ? simx_opmode_streaming : simx_opmode_buffer);
	if (error != simx_return_ok && error != simx_error_novalue_flag)
	{
		if (isStreamingInitialized == false)
		{
			throw std::string("Unable to set up object joint streaming.");
		}
		else
		{
			throw std::string("Unable to fetch joint position.");
		}
	}

	return pos;
}

double VrepBasicConnector::GetMotorTorque(int handle, bool isStreamingInitialized) const
{
	simxFloat torque;
	simxInt error = simxJointGetForce(m_ClientID, handle, &torque, isStreamingInitialized == false ? simx_opmode_streaming : simx_opmode_buffer);

	if (error != simx_return_ok && error != simx_error_novalue_flag)
	{
		if (isStreamingInitialized == false)
		{
			throw std::string("Unable to set up joint torque streaming.");
		}
		else
		{
			throw std::string("Unable to fetch joint torque.");
		}
	}

	return torque;
}

void VrepBasicConnector::GetObjectVelocity(int handle, std::vector<double>& linearVelocity, std::vector<double>& angularVelocty, bool isStreamingInitialized) const
{
	float linV[3] = { 0 };
	float angV[3] = { 0 };

	simxInt error = simxGetObjectVelocity(m_ClientID, handle, linV, angV, isStreamingInitialized == false ? simx_opmode_streaming : simx_opmode_buffer);

	if (error != simx_return_ok && error != simx_error_novalue_flag)
	{
		if (isStreamingInitialized == false)
		{
			throw std::string("Unable to set up object velocity streaming.");
		}
		else
		{
			throw std::string("Unable to fetch object velocity.");
		}
	}

	linearVelocity.clear();
	linearVelocity.assign(linV, linV + 3);

	angularVelocty.clear();
	angularVelocty.assign(angV, angV + 3);
}

std::vector<double> VrepBasicConnector::GetObjectPosition(int handle, int relativeToObjectHandle, bool isStreamingInitialized) const
{
	float result[3] = { 0 };

	simxInt error = simxGetObjectPosition(m_ClientID, handle, relativeToObjectHandle, result, isStreamingInitialized == false ? simx_opmode_streaming : simx_opmode_buffer);

	if (error != simx_return_ok && error != simx_error_novalue_flag)
	{
		if (isStreamingInitialized == false)
		{
			throw std::string("Unable to set up object position streaming.");
		}
		else
		{
			throw std::string("Unable to fetch object position.");
		}
	}

	return std::vector<double>(result, result + sizeof(result) / sizeof(float));
}

std::vector<double> VrepBasicConnector::GetObjectOrientation(int handle, int relativeToObjectHandle, bool isStreamingInitialized) const
{
	float result[3] = { 0 };

	simxInt error = simxGetObjectOrientation(m_ClientID, handle, relativeToObjectHandle, result, isStreamingInitialized == false ? simx_opmode_streaming : simx_opmode_buffer);

	if (error != simx_return_ok && error != simx_error_novalue_flag)
	{
		if (isStreamingInitialized == false)
		{
			throw std::string("Unable to set up object orientation streaming.");
		}
		else
		{
			throw std::string("Unable to fetch object orientation.");
		}
	}

	return std::vector<double>(result, result + sizeof(result) / sizeof(float));
}

void VrepBasicConnector::GetObjectGroupData(int objectType, int dataType,
	std::vector<int>& handles, std::vector<int>& ints, std::vector<double>& doubles, std::vector<std::string>& strings,
	bool isStreamingInitialized) const
{
	handles.clear();
	ints.clear();
	doubles.clear();
	strings.clear();

	int handlesRawCount = 0;
	int* handlesRaw = new int;

	int intDataCount = 0;
	int* intData = new int;

	int floatDataCount = 0;
	float* floatData = new float;

	int stringDataCount = 0;
	char* stringData = new simxChar;

	simxInt error = simxGetObjectGroupData(m_ClientID,
		objectType,
		dataType,
		&handlesRawCount, &handlesRaw,
		&intDataCount, &intData,
		&floatDataCount, &floatData,
		&stringDataCount, &stringData,
		isStreamingInitialized == false ? simx_opmode_streaming : simx_opmode_buffer);

	if (error != simx_return_ok && error != simx_error_novalue_flag)
	{
		throw std::string("Unable to get object map.");
	}

	handles.assign(handlesRaw, handlesRaw + handlesRawCount);
	ints.assign(intData, intData + intDataCount);
	doubles.assign(floatData, floatData + floatDataCount);

	if (stringDataCount > 0)
	{
		// strings are delimited with '\0'

		strings.reserve(stringDataCount);
		int numStrings = 0;
		int currentIndex = 0;
		std::string currentName = "";

		while (numStrings != stringDataCount)
		{
			if (stringData[currentIndex] == 0)
			{
				strings.push_back(currentName);

				numStrings++;

				currentName.clear();
			}
			else
			{
				currentName.append(1, stringData[currentIndex]);
			}

			currentIndex++;
		}
	}
}

void VrepBasicConnector::GetObjectGroupData(int objectType, int dataType,
	std::vector<int>& handles, std::vector<int>& ints, std::vector<double>& doubles, std::vector<std::string>& strings) const
{
	handles.clear();
	ints.clear();
	doubles.clear();
	strings.clear();

	int handlesRawCount = 0;
	int* handlesRaw = new int;

	int intDataCount = 0;
	int* intData = new int;

	int floatDataCount = 0;
	float* floatData = new float;

	int stringDataCount = 0;
	char* stringData = new simxChar;

	simxInt error = simxGetObjectGroupData(m_ClientID,
		objectType,
		dataType,
		&handlesRawCount, &handlesRaw,
		&intDataCount, &intData,
		&floatDataCount, &floatData,
		&stringDataCount, &stringData,
		simx_opmode_oneshot_wait);

	if (error != simx_return_ok && error != simx_error_novalue_flag)
	{
		throw std::string("Unable to get object map.");
	}

	handles.assign(handlesRaw, handlesRaw + handlesRawCount);
	ints.assign(intData, intData + intDataCount);
	doubles.assign(floatData, floatData + floatDataCount);

	if (stringDataCount > 0)
	{
		// strings are delimited with '\0'

		strings.reserve(stringDataCount);
		int numStrings = 0;
		int currentIndex = 0;
		std::string currentName = "";

		while (numStrings != stringDataCount)
		{
			if (stringData[currentIndex] == 0)
			{
				strings.push_back(currentName);

				numStrings++;

				currentName.clear();
			}
			else
			{
				currentName.append(1, stringData[currentIndex]);
			}

			currentIndex++;
		}
	}
}

void VrepBasicConnector::GetProximitySensorData(int handle, bool& detectionState, std::vector<double>& detectedPoint, int& detectedObjectHandle, std::vector<double>& detectedSurfaceNormalVector, bool isStreamingInitialized)
{
	unsigned char state = 0;
	float point[3] = { 0 };
	int objectHandle = -1;
	float surfaceNormalVector[3] = { 0 };

	detectedPoint.clear();
	detectedSurfaceNormalVector.clear();

	// detectedPoint: pointer to 3 values receiving the detected point coordinates(relative to the sensor reference frame). Can be NULL.

	simxInt error = simxReadProximitySensor(m_ClientID, handle, &state, point, &objectHandle, surfaceNormalVector,
		isStreamingInitialized == false ? simx_opmode_streaming : simx_opmode_buffer);

	if (error == simx_return_ok && state == 1)
	{
		detectionState = true;
		detectedObjectHandle = objectHandle;
		detectedPoint.assign(point, point + 3);
		detectedSurfaceNormalVector.assign(surfaceNormalVector, surfaceNormalVector + 3);

	}
	else
	{
		detectionState = false;
	}
}
void VrepBasicConnector::GetVisionSensorData(int handle, int& height, int& width, std::vector<unsigned char>& imageVector, char options, bool isStreamingInitialized)
{	
	unsigned char* img = new unsigned char;
	int* res = new int[2];
	res[0] = res[1] = 0;	

	simxInt error = simxGetVisionSensorImage(m_ClientID, handle, res, &img, options, isStreamingInitialized == false ? simx_opmode_streaming : simx_opmode_buffer);

	if (error != simx_return_ok && error != simx_error_novalue_flag)
	{
		throw std::string("Vision sensor image read error.");
	}

	height = res[0];
	width = res[1];

	// TODO: magic number 49535?
	// width * height * 3 = 49152
	// difference: 383
	if (error == simx_return_ok)
	{
		imageVector.clear();
		imageVector.assign(img, img + 49535 + 1);
	}
	
}