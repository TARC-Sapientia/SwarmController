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


#include "VideoPlayer.h"

#include <iostream>
#include <boost/lexical_cast.hpp>

using namespace std;
using namespace boost;

namespace VideoHandler
{	
	VideoPlayer::VideoPlayer() 
	{

	}

	VideoPlayer::~VideoPlayer() {

	}

	IVideoPlayer* getVideoPlayer() {
		return new VideoPlayer();
	}
	
	void VideoPlayer::addVideoPlayback(unsigned int frameStreamId, unsigned int maxDecodingFrameCount) {
		m_videoPlaybacks.insert(make_pair(
			frameStreamId, 
			boost::shared_ptr<VideoPlayback>(new VideoPlayback(*this, frameStreamId, maxDecodingFrameCount))));
	}
	
	void VideoPlayer::pushEncodedFrame(unsigned int frameStreamId, boost::shared_array<char> data, const unsigned int dataLength) {
		unique_lock<boost::mutex> lock(m_access);
		if (m_videoPlaybacks.find(frameStreamId) != m_videoPlaybacks.end())
			m_videoPlaybacks[frameStreamId]->pushVideoFrame(data, dataLength);
	}

} // end namespace VideoHandler
