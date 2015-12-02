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


#ifndef IMAGEPROCESSOR_H
#define IMAGEPROCESSOR_H

#include <vector>
#include <map>

#include <boost\shared_ptr.hpp>
#include "IVideoPlayer.h"

#include "opencv\cxcore.h"
#include "opencv\highgui.h"
#include "opencv2\highgui\highgui.hpp"
#include "opencv2\imgproc\imgproc.hpp"
#include <opencv2\objdetect\objdetect.hpp>
#include <opencv2\features2d\features2d.hpp>
#include <opencv2\calib3d\calib3d.hpp>
#include <opencv2\nonfree\nonfree.hpp>

using namespace std;
using namespace cv;

namespace ImageProcessing
{
	class ImageProcessor
	{
	public:
		// Constructor
		// Initializing predefined parameters
		ImageProcessor();

		// Destructor
		~ImageProcessor();

		// Process and show the image
		map<int, map<string, double>> ProcessImage(std::vector<unsigned char>& imageVector, int height, int width, bool showImage);

		// Use this objects data only if this is true
		bool IsDetecting();

	private:
		// Slices image in equal parts
		vector<Mat> SliceImage(vector<unsigned char>& imageVector, int width, int height, bool displayImages);

		// Detect blobs in images
		map<string, double> DetectBlobs(Mat& image);

		// Display the read image
		void DisplayImage(bool display, Mat& frame);

		// Define objects used for processing image
		Mat mask, hsvImage, image;

		// Simple blob detector
		SimpleBlobDetector::Params params;

		// Found kepoints in image
		vector<KeyPoint> keypoints;

		// Currently detected points in image slice 
		// Saving coordinates of the point and the size of the detected blob
		map<string, double> m_DetectedPoints;

		// Currently detected points list from all the image slices
		// Image slice number defined by the int in the map
		map<int, map<string, double>> m_DetectedPointList;

		// Detection is only valid, if this is true
		bool m_IsDetecting;

		// Video player for showing current image read from epuck
		VideoHandler::VideoPlayerPtr m_VideoPlayer;
	};

	typedef boost::shared_ptr<ImageProcessor> ImageProcessorPtr;
}
#endif // IMAGEPROCESSOR_H