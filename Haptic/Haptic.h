#ifndef HAPTICDEVICESTATE_H
#define HAPTICDEVICESTATE_H

namespace Haptics
{
	enum HapticDevice
	{
		PHANToM_H1 = 0,
		PHANToM_H2 = 1
	};

	enum ButtonState
	{
		None = 0,
		DarkPressed = 1,
		LighPressed = 2,
		BothPressed = 3
	};


	struct HapticData
	{
		double position[3];			// mm
		double velocity[3];			// mm/s
		double jointAngles[3];		// rad;
		double gimbalAngles[3];		// rad;
		double force[3];			// N

		ButtonState buttonState;				// 1, 2
	};

} // end namespace
	
#endif //HAPTICDEVICESTATE_H
