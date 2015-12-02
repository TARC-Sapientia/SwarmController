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


#include "Core.h" // pair header

// Default thread no. If 0 then the number of hardware concurrency will be used
#define DEFAULT_THREAD_NO 0

using namespace std;
using namespace boost;
using namespace boost::asio;

namespace Utils
{
#define MINIMUM_THREAD_NO 4

	int Core::threadNo = DEFAULT_THREAD_NO;

	Core* Core::m_instance = NULL;

	Core& Core::get()
	{
		static recursive_mutex access;
		if (m_instance == NULL)
		{
			recursive_mutex::scoped_lock lock(access);
			if (m_instance == NULL)
				m_instance = new Core();
		}
		return *m_instance;
	}

	void Core::cleanup()
	{
		delete m_instance;
		m_instance = NULL;
	}

	boost::asio::io_service& Core::getIoService()
	{
		return *m_pioService;
	}

	void Core::stop()
	{
		m_pioService->stop();

		// wait for every thread to terminate and delete it
		list<thread*>::iterator it;
		for (it = m_pThreads->begin(); it != m_pThreads->end(); it++)
		{
			(*it)->join();
			delete *it;
		}

		// clear the list
		m_pThreads->clear();
	}

	Core::Core()
	{
		int threads = threadNo;

		// if zero then use hardware concurrency
		if (threads == 0)
		{
			threads = thread::hardware_concurrency();
		}

		// if less then MINIMUM_THREAD_NO use MINIMUM_THREAD_NO
		if (threads < MINIMUM_THREAD_NO)
		{
			threads = MINIMUM_THREAD_NO;
		}

		m_pioService = new boost::asio::io_service();
		m_pThreads = new list<boost::thread*>();
		// create the threads
		while (threads > 0)
		{
			m_pThreads->push_back(new thread(boost::bind(&Core::run, this)));
			threads--;
		}

	}

	Core::~Core()
	{
		m_pThreads->clear();
		delete m_pThreads;
		delete m_pioService;

		delete m_instance;
	}

	void Core::run()
	{
		io_service::work work(*m_pioService);
		m_pioService->run();
	}
}