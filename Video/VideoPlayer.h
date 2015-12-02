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


#ifndef VIDEO_PLAYER_H
#define VIDEO_PLAYER_H

#include "IVideoPlayer.h"
#include "VideoPlayback.h"

#include <queue>
#include <map>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/types_c.h>

namespace VideoHandler
{
	class VideoPlayer : public IVideoPlayer
	{
	public:

		VideoPlayer();
		~VideoPlayer();
	
		virtual void pushEncodedFrame(unsigned int frameStreamId, boost::shared_array<char> data, const unsigned int dataLength);

		virtual void addVideoPlayback(unsigned int frameStreamId, unsigned int maxDecodingFrameCount);

	private:

		std::map<unsigned int, boost::shared_ptr<VideoPlayback> > m_videoPlaybacks;
		boost::mutex m_access;
	};

} // end namespace VideoHandler

#endif //VIDEO_PLAYER_H
