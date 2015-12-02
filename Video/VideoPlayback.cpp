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


#include "VideoPlayback.h"
#include "VideoPlayer.h"

//#include "Core.h" //$(SolutionDir)Communication\include;
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/types_c.h>

#include <boost/lexical_cast.hpp>

using namespace std;
using namespace boost;

namespace VideoHandler
{	
	VideoPlayback::VideoPlayback(VideoPlayer& videoPlayer, unsigned int frameStreamId, unsigned int videoFramesMaxLength) :
	  m_videoPlayer(&videoPlayer),
	  m_frameStreamId(frameStreamId),
	  m_videoFramesMaxLength(videoFramesMaxLength),
	  m_threadRunning(true),
	  m_windowName("Camera " + boost::lexical_cast<string>(m_frameStreamId)),
	  m_videoPlaybackThread(new boost::thread(boost::bind(&VideoPlayback::decodeVideoFrame, this))) {

	}

	VideoPlayback::~VideoPlayback() {
		{
			unique_lock<mutex> lock(m_access);
			m_threadRunning = false;
		}
		m_condition.notify_all();
		m_videoPlaybackThread->join();

		m_videoFrames.clear();
	}
	
	void VideoPlayback::pushVideoFrame(boost::shared_array<char> data, const unsigned int dataLength) {
		bool frameThrown = false;
		unsigned int size = 0;
		{
			unique_lock<mutex> lock(m_access);
			m_videoFrames.push_back(EncodedVideoFrame(data, dataLength));
			if (m_videoFrames.size() > m_videoFramesMaxLength) {
				frameThrown = true;
				m_videoFrames.pop_front();
			}
			size = m_videoFrames.size();
			m_condition.notify_all();
		}

		if (frameThrown)
			cout << "FrameStream " << m_frameStreamId << ": decode list full. Oldest frame thrown." << endl;
	}

	void VideoPlayback::decodeVideoFrame()
	{
		try {
			unique_lock<mutex> lock(m_access);
			while (m_threadRunning) {
				if (m_videoFrames.size() == 0) {
					m_condition.wait(lock);
					continue;
				}

				EncodedVideoFrame encodedFrame = m_videoFrames.front();
				m_videoFrames.pop_front();
				unsigned int size = m_videoFrames.size();
				lock.unlock();

				// decode the frame
				vector<char> buf(encodedFrame.data.get(), encodedFrame.data.get() + encodedFrame.dataLength);
				cv::Mat img(buf);
				cv::Mat frame = imdecode(img, -1);

				// display the decoded frame
				try {
					imshow(m_windowName, frame);
				} 
				catch (std::exception& e) {
					cout << "Imshow Exception caught:" << endl << e.what() << endl;
				}
				cv::waitKey(1);
				lock.lock();
			}
		}
		catch (thread_interrupted&) {
		}
	}

} // end namespace VideoHandler
