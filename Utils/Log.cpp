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


#include "Log.h"
#include <iostream>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/time_facet.hpp>
#include <boost/filesystem.hpp>

using namespace std;

namespace Utils
{
	Log::Log(const string& path, const string& fileName)
	{
		// Check if the directory exists
		boost::filesystem::path dir(path);
		if (boost::filesystem::exists(path) || boost::filesystem::create_directory(dir))
		{
			// Get the current time
			boost::posix_time::ptime currentTime = boost::posix_time::microsec_clock::local_time();

			stringstream ss;

			// Get current time
			//date_time::time_facet<posix_time::ptime, char>* facetPtr = new date_time::time_facet<posix_time::ptime, char>("%Y-%b-%d-%H-%M-%S");
			//ss.imbue(locale(ss.getloc(), facetPtr));

			ss << fileName 
				//<< "-" << currentTime // Removed current time->easier processed in matlab
				<< ".txt";

			// Create the file
			string fileFullPath = path + "/" + ss.str();
			logFile.open(fileFullPath.c_str(), ios::in | ios_base::trunc);
			
			if (logFile.is_open() == false)
			{
				throw ("Unable to create the log file: " + fileFullPath);
			}
		}
		else
		{
			throw ("Unable to create folder: " + path);
		}
	}


	void Log::Write(const string& message)
	{
		if (logFile && logFile.is_open())
		{
			logFile << message;
		}
	}


	Log::~Log()
	{
		logFile.close();
	}

} // end Utils namespace
