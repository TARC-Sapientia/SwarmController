#include "YouBot.h"
#include <string>
#include <iostream>

using namespace VrepAPI;
using namespace Utils;
using namespace std;

YouBot::YouBot(std::string name)
	: YouBot(-1, name)
{
}

YouBot::YouBot(int handle, std::string name)
	: Robot(handle, name),
	m_VelocityFrontLeft(0.00),
	m_VelocityFrontRight(0.00),
	m_VelocityRearLeft(0.00),
	m_VelocityRearRight(0.00),
	m_WheelRadius(0.0475),
	m_HalfAxeLength(0.15),
	m_HalfFrontAxeLength(0.235),
	m_Logger(new Log("Log", name))
{
}

YouBot::~YouBot()
{
}

void YouBot::Load(VrepConnectorPtr& vrepConnectorPtr)
{
	for (auto component : m_Components)
	{
		m_Motors[RollingJoint_FrontLeft] = boost::dynamic_pointer_cast<Motor>(GetComponent("rollingJoint_fl"));
		m_Motors[RollingJoint_FrontRight] = boost::dynamic_pointer_cast<Motor>(GetComponent("rollingJoint_fr"));
		m_Motors[RollingJoint_RearLeft] = boost::dynamic_pointer_cast<Motor>(GetComponent("rollingJoint_rl"));
		m_Motors[RollingJoint_RearRight] = boost::dynamic_pointer_cast<Motor>(GetComponent("rollingJoint_rr"));
		m_Motors[ArmJoint0] = boost::dynamic_pointer_cast<Motor>(GetComponent("youBotArmJoint0"));
		m_Motors[ArmJoint1] = boost::dynamic_pointer_cast<Motor>(GetComponent("youBotArmJoint1"));
		m_Motors[ArmJoint2] = boost::dynamic_pointer_cast<Motor>(GetComponent("youBotArmJoint2"));
		m_Motors[ArmJoint3] = boost::dynamic_pointer_cast<Motor>(GetComponent("youBotArmJoint3"));
		m_Motors[ArmJoint4] = boost::dynamic_pointer_cast<Motor>(GetComponent("youBotArmJoint4"));
		m_Motors[GripperJoint1] = boost::dynamic_pointer_cast<Motor>(GetComponent("youBotGripperJoint1"));
		m_Motors[GripperJoint2] = boost::dynamic_pointer_cast<Motor>(GetComponent("youBotGripperJoint2"));

		// Load each component
		if (auto object = boost::dynamic_pointer_cast<VrepObject>(component))
		{
			object->Load(vrepConnectorPtr);
		}
	}
}

void YouBot::Update(VrepConnectorPtr& vrepConnectorPtr)
{
	// Update motors
	for (auto motor : m_Motors)
	{
		motor.second->Update(vrepConnectorPtr);
	}

	// Update position and orientation
	m_Position = vrepConnectorPtr->GetObjectPosition(m_Handle, -1, m_IsStreamingInitialized);
	m_Orientation = vrepConnectorPtr->GetObjectOrientation(m_Handle, -1, m_IsStreamingInitialized);

	// Update velocity
	vrepConnectorPtr->GetObjectVelocity(m_Handle, m_VelocityRawLinear, m_VelocityRawAngular, m_IsStreamingInitialized);

//	//cout << "v: " << m_VelocityLinear[0] << '\t' << m_VelocityLinear[1] << '\t' << m_VelocityLinear[2] << '\t';
//	//cout << "w: " << m_VelocityAngular[0] << '\t' << m_VelocityAngular[1] << '\t' << m_VelocityAngular[2] << endl;	

	if (m_IsStreamingInitialized == true)
	{
		// Log status
		m_Logger->Write(
			"v: " + to_string(Vector2(m_VelocityRawLinear[0], m_VelocityRawLinear[1]).Magnitude()) + "\tw: 0.000" // in V-Rep
			+ "\tvRef: " + to_string(m_SpeedReference.x) + "\twRef: " + to_string(m_SpeedReference.y)  // calculated velocity
			+ "\tXg: " + to_string(m_Position[0]) + "\tYg: " + to_string(m_Position[1]) + "\tZg: " + to_string(m_Position[2])
			+ "\tAlpha: " + to_string(m_Orientation[0]) + "\tBeta: " + to_string(m_Orientation[1]) + "\tGamma: " + to_string(m_Orientation[2])
			+ "\n");
	}

	if (m_IsStreamingInitialized == false) m_IsStreamingInitialized = true;
}

Vector2 YouBot::CalculateSpeed()
{
	return Vector2();
}

void YouBot::SetWheelSpeed(Vector2 robotSpeed)
{
}

void YouBot::CalculateWheelSpeed(double hapticPosZ, double hapticGimbalAngle, double hapticPosX)
{
	hapticPosZ *= .01;
	hapticGimbalAngle *= -.1;
	hapticPosX *= .05;

	m_VelocityFrontLeft = (1 / m_WheelRadius) * (hapticPosZ + hapticGimbalAngle + (-1 * (m_HalfAxeLength + m_HalfFrontAxeLength) * hapticPosX));
	m_VelocityFrontRight = (1 / m_WheelRadius) * (hapticPosZ - hapticGimbalAngle + ((m_HalfAxeLength + m_HalfFrontAxeLength) * hapticPosX));
	m_VelocityRearLeft = (1 / m_WheelRadius) * (hapticPosZ - hapticGimbalAngle + (-1 * (m_HalfAxeLength + m_HalfFrontAxeLength) * hapticPosX));
	m_VelocityRearRight = (1 / m_WheelRadius) * (hapticPosZ + hapticGimbalAngle + ((m_HalfAxeLength + m_HalfFrontAxeLength) * hapticPosX));

	m_SpeedReference.x = (m_WheelRadius / 4) * (m_VelocityFrontLeft + m_VelocityFrontRight + m_VelocityRearLeft + m_VelocityRearRight);
	m_SpeedReference.y = (m_WheelRadius / 4) * (m_VelocityFrontLeft - m_VelocityFrontRight - m_VelocityRearLeft + m_VelocityRearRight);
}

void YouBot::SetWheelSpeed(double hapticPosZ, double hapticGimbalAngle, double hapticPosX)
{
	CalculateWheelSpeed(hapticPosZ, hapticGimbalAngle, hapticPosX);

	m_Motors[RollingJoint_FrontLeft]->SetSpeed(m_VelocityFrontLeft);
	m_Motors[RollingJoint_FrontRight]->SetSpeed(m_VelocityFrontRight);
	m_Motors[RollingJoint_RearLeft]->SetSpeed(m_VelocityRearLeft);
	m_Motors[RollingJoint_RearRight]->SetSpeed(m_VelocityRearRight);
}
