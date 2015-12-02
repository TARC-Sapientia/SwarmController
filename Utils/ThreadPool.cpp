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


#include <list>
#include <boost/thread/thread.hpp>
#include <boost/function/function0.hpp>
#include <boost/asio/io_service.hpp>
#include "ThreadPool.hpp"

using namespace std;
using namespace boost;

namespace Utils
{
	struct ThreadPool::Data {

		/** The used io_service */
		asio::io_service ioService;

		/** Threads of this pool */
		list<thread*> threads;

		/** Ensures work */
		asio::io_service::work* work;
	};

	ThreadPool::ThreadPool(unsigned int threadsCount) throw () :
		d(new Data) {

		d->work = new asio::io_service::work(d->ioService);

		// If zero then use harware concurrency.
		if (threadsCount == 0)
			threadsCount = thread::hardware_concurrency();
		// If still 0 then use one thread only.
		if (threadsCount == 0)
			threadsCount = 1;
		// Create the threads.
		while (threadsCount > 0) {
			d->threads.push_back(new thread(boost::bind(&ThreadPool::run, this)));
			threadsCount--;
		}
	}

	ThreadPool::~ThreadPool() throw () {
		// Delete the dummy work so the threads terminate.
		delete d->work;

		// Wait for every thread to terminate and delete it.
		list<thread*>::iterator it;
		for (it = d->threads.begin(); it != d->threads.end(); it++) {
			(*it)->join();
			delete *it;
		}
	}

	void ThreadPool::dispatch(Handler handler) {
		d->ioService.dispatch(handler);
	}

	void ThreadPool::post(Handler handler) {
		d->ioService.post(handler);
	}

	int ThreadPool::getThreadNumber() throw () {
		return d->threads.size();
	}

	asio::io_service& ThreadPool::getIoService() {
		return d->ioService;
	}

	void ThreadPool::run() {
		d->ioService.run();
	}
}