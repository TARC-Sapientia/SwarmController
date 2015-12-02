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


#ifndef VIDEO_PLAYBACK_H
#define VIDEO_PLAYBACK_H

#include <iostream>
#include <string>
#include <memory>
#include <boost/shared_array.hpp>
#include <boost/thread.hpp>
#include <list>

namespace VideoHandler
{
	class VideoPlayer;

	class VideoPlayback
	{
	public:

		typedef struct EncodedVideoFrame {
			boost::shared_array<char> data;
			const unsigned int dataLength;

			EncodedVideoFrame() : data(), dataLength(0) { }

			EncodedVideoFrame(boost::shared_array<char> _data, const unsigned int _dataLength)
				: data(_data), dataLength(_dataLength)
			{

			}

			/*EncodedVideoFrame& operator=(const EncodedVideoFrame& frame)
			{
				if (this != &frame) { 
					data = frame.data;
					dataLength = frame.dataLength;
				}
				return *this;
			}*/
		} EncodedVideoFrame;

		VideoPlayback(VideoPlayer& videoPlayer, unsigned int frameStreamId, unsigned int videoFramesMaxLength);
		~VideoPlayback();

		void pushVideoFrame(boost::shared_array<char> data, const unsigned int dataLength);

	private:

		void decodeVideoFrame();

		VideoPlayer* m_videoPlayer;
		unsigned int m_frameStreamId;
		std::string m_windowName;
		std::auto_ptr<boost::thread> m_videoPlaybackThread;

		bool m_threadRunning;
		std::list<EncodedVideoFrame> m_videoFrames;
		unsigned int m_videoFramesMaxLength;

		boost::mutex m_access;
		boost::condition_variable m_condition;
	};

} // end namespace VideoHandler

#endif //VIDEO_PLAYBACK_H
