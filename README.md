# SwarmController
### Teleoperation of robot swarms in a V-Rep based simulation environment

# General description of the system
The simulated multi-robot system is composed of a leader robot and the tracking swarm. The task of the swarm is to follow the leader robot while it maintains the formation and avoids collisions. The swarm is composed of identical members. Each swarm member scans for objects in its field of view using proximity and color sensors. Based on sensor information attractive and repulsive forces are generated and summed from which the linear and angular robot speeds are calculated. The motion of the target robot is controlled by the human operator using a Sensable Phantom Omni haptic device.

The client application, responsible for the leader control connects to the V-REP remote C++ API interface. For the leader robot the KUKA youBot was selected, while the swarm is composed of e-puck robots. Both robot models are provided in V-REP which also handles both the robot proximity and color sensors.

For demos and publications, acknowledgement can be found on the project website:
http://www.ms.sapientia.ro/~martonl/MartonL_Research_TE.htm

# License

Teleoperation System for Mobile Manipulators Framework

Copyright (C) 2015 RECONFIGURABLE CONTROL OF ROBOTIC SYSTEMS OVER NETWORKS Project

Robotics and Control Systems Laboratory, Department of Electrical Engineering, Sapientia Hungarian University of Transylvania

This program is a free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this program. If not, see http://www.gnu.org/licenses/.

# System requirements:
##Install build tools:
  1. Boost libraries (>= 1.55) http://www.boost.org/  
  2. OpenCV (>= 2.4.8) http://opencv.org/
  3. V-REP (>= 3.2.1) http://www.coppeliarobotics.com/
  4. Git client

# Running the system:
  1. Start the V-REP simulator and load your scene. A demo scene is provided in the \scenes folder.
  2. In the Master.cpp provide the correct address (IP and port) of the V-REP server.
  3. Start the SwarmController application.
  4. The leader motion is enabled by pressing the dark button of the haptic device.
  5. The position of the haptic device (Forward-Backward / Left-Right) generates the velocities of the leader robot (Longitudinal / Transversal).
  6. Whenever the leader robot is in the sight of some swarm members, the swarm follows the leader.
