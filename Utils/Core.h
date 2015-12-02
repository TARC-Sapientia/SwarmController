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


#pragma once
#ifndef CORE_H
#define CORE_H

#ifdef _WIN32
#ifndef _WIN32_WINNT
#define _WIN32_WINNT 0x0501
#endif
#endif // _WIN32

#include <boost/asio.hpp>
#include <boost/thread.hpp>

namespace Utils
{
	// Core singleton class.
	// Contains the required resources for network communication.
	class Core
	{
	public:

		// The number of threads to be used by the network core.
		// If 0 then the thread number will be equal with the number of processor cores.
		static int threadNo;

		// Get the core instance.
		static Core& get();

		// Cleanup to be 
		static void cleanup();

		// Retrieve the io service used by the core
		// @return the io service.
		boost::asio::io_service& getIoService();

		// Retrieve the number of threads used for network io services
		// @return the number of threads
		unsigned int getThreadNo();

		// Stop the core
		void stop();

	protected:

		// Constructor 
		Core();

		// Destructor 
		~Core();

		// IO thread function
		void run();

		// The singleton core instance
		static Core* m_instance;

		// The io service
		boost::asio::io_service* m_pioService;

		// IO thread pool
		std::list<boost::thread*>* m_pThreads;
	};
}

#endif // CORE_H
