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


#ifndef UTILS_THREADPOOL_HPP
#define UTILS_THREADPOOL_HPP

#include <memory>
#include <boost/noncopyable.hpp>

// boost forward declarations
namespace boost {
  template<typename Signature> class function0;

  namespace asio {
    class io_service;
  }
}

namespace Utils {

	/**
	 * Implementation of a thread pool using asio::io_service.
	 * Handlers can be given for execution using dispatch and post functions.
	 */
	class ThreadPool : boost::noncopyable {

	public:

		typedef boost::function0<void> Handler;

		/**
		 * Constructor
		 */
		ThreadPool(unsigned int threadsCount = 0) throw();

		/**
		 * Destructor.
		 * Also wait for all the posted handlers to be executed.
		 */
		~ThreadPool() throw();

		/**
		 * Dispatch a handler to be executed.
		 * If the calling thread is one belonging to the pool then it will be executed
		 * by the calling thread otherwise inside this function. Otherwise it will be
		 * posted to be executed by one of the thread pool.
		 */
		void dispatch(Handler handler);

		/**
		 * Post a handler to be executed by one of the pool's thread.
		 * The function returns immediately.
		 */
		void post(Handler handler);

		/**
		 * Returns the number of threads in the threads list
		 * @return the number of threads in the threads list
		 */
		int getThreadNumber() throw();

		/**
		 * Retrieve the I/O service associated with the pool
		 */
		boost::asio::io_service& getIoService();

	protected:

		/** The thread function */
		void run();

		// Opaque data type
		struct Data;
		const std::auto_ptr<Data> d;
	};

}

#endif // UTILS_THREADPOOL_HPP
