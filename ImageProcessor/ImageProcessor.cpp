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


#include "ImageProcessor.h"

#include <iostream>
#define D_SCL_SECURE_NO_WARNINGS

using namespace ImageProcessing;

ImageProcessor::ImageProcessor()
	: m_VideoPlayer(VideoHandler::getVideoPlayer()),
	// By default, the robot is not detecting anything
	m_IsDetecting(false),
	m_DetectedPoints({ { "x", .0 }, { "y", .0 }, { "size", .0 } })
{
	// Stream ID for video player
	auto streamId = 0;
	// Video player max decoding queue length 
	auto maxDecodingQueueLength = 2;
	// Video player
	m_VideoPlayer->addVideoPlayback(streamId, maxDecodingQueueLength);
	// Set minimum distance between detected blobs in image. Unit: 
	params.minDistBetweenBlobs = 10.0f;
	// Set filter by inertia to false, we filter by area
	params.filterByInertia = false;
	// Set filter by convexity to false, we filter by area
	params.filterByConvexity = false;
	// Set filter by color to false, we filter by area
	params.filterByColor = false;
	// Set filter by circularity to false, we filter by area
	params.filterByCircularity = false;
	// Set filter by area to true, we want to filter by area
	params.filterByArea = true;
	// Set minimum area to be detected in image. Unit: 
	params.minArea = 200.0f;
	// Set maximum area to be detected in image. Unit: 
	params.maxArea = 10000.0f;
	// Set minimum treshold. Unit: 
	params.minThreshold = 150.0f;

	m_VideoPlayer->addVideoPlayback(0, 2);
}

ImageProcessor::~ImageProcessor()
{
}

map<int, map<string, double>> ImageProcessor::ProcessImage(std::vector<unsigned char>& imageVector, int height, int width, bool showImage)
{
	// Set detection to false by default
	// It will change if the robot is detecting something
	m_IsDetecting = false;

	// Order of the image slice 
	// Number between [0..3], but this can change
	int imageCounter = 0;
	
	// Slice the image received from the ePuck
	auto imageSlices = SliceImage(imageVector, width, height, showImage);
	
	// Clear earlier detected points
	m_DetectedPointList.clear();
	m_DetectedPointList.empty();
	
	// Prepare the response
	for (auto it : imageSlices)
	{
		// Saving in map<int, map<string, double>> to know in which slice we have the detected object
		m_DetectedPointList.insert(make_pair(imageCounter++, DetectBlobs(it)));
	}

	return m_DetectedPointList;
}

vector<Mat> ImageProcessor::SliceImage(vector<unsigned char>& imageVector, int width, int height, bool displayImages)
{
	// Initialize the image slices
	Mat firstPart(height, width / 2, CV_8UC3);
	Mat secondPart(height, width / 2, CV_8UC3);
	Mat thirdPart(height, width / 2, CV_8UC3);
	Mat ocvImage(height, width, CV_8UC3);

	// Initialize a vector in which we'll save all the slices
	vector<Mat> slices;
	unsigned int j1, j2, j3;

	// Porcess the image, create the image slices
	for (auto i = 0; i < (unsigned int)height; i++)
	{
		j1 = 0; j2 = 0; j3 = 0;
		for (auto j = 0; j < (unsigned int)width; j++)
		{
			// Index of the pixel we want to save
			auto pixelIndex = 3 * ((width - i) * height + j);

			// Extract the R,G,B pixels from the image
			auto r = cvRound(255 * imageVector[pixelIndex + 0]);
			auto g = cvRound(255 * imageVector[pixelIndex + 1]);
			auto b = cvRound(255 * imageVector[pixelIndex + 2]);

			// Create a pixel from the RGB colors
			Vec3b pixel = Vec3b((uchar)r, (uchar)g, (uchar)b);

			// Set the the pixel in the original image
			ocvImage.at<Vec3b>(i, j) = pixel;

			// Set the pixel for the image slices
			if (j < (unsigned int)(width / 2)) {
				firstPart.at<Vec3b>(i, j1) = pixel;
				j1++;
			}
			else
			{
				thirdPart.at<Vec3b>(i, j3) = pixel;
				j3++;
			}
			if ((j >= (unsigned int)(width / 4)) && (j < (unsigned int)(3 * width / 4)))
			{
				secondPart.at<Vec3b>(i, j2) = pixel;
				j2++;
			}
		}
	}
	// Called only if we want to display the obtained image
	DisplayImage(displayImages, ocvImage);

	// Save the slices
	slices.push_back(firstPart);
	slices.push_back(secondPart);
	slices.push_back(thirdPart);

	return slices;
}

map<string, double> ImageProcessor::DetectBlobs(Mat& image)
{
	// Initialize Simple Blob Detector with the initialized parameters
	SimpleBlobDetector blob_detector(params);
	// Convert the image from RGB to HSV (RedGreenBlue -> HueSaturationValue)
	cvtColor(image, hsvImage, CV_RGB2HSV);
	// Create a black-white mask: 
	// white will be the color we select with the range of the Scalar values
	// black will be the rest of the image
	inRange(hsvImage, Scalar(20, 150, 150), Scalar(130, 250, 250), mask);
	// Detect the blobs in the mask, save the keypoints
	blob_detector.detect(mask, keypoints);
	// Coordinates and size of the detected point
	double X = 0; double Y = 0; double size = 0;

	// Save the biggest blob size
	// Prepare coordinates for an average coordinate between the detected points
	for (auto key : keypoints)
	{
		X += key.pt.x;
		Y += key.pt.y;
		if (key.size > size) size = key.size;
	}

	// Calculate the average coordinate for the detected points
	if (keypoints.size() > 0)
	{
		X /= keypoints.size();
		Y /= keypoints.size();
		// Set IsDetecting to true only if we have kaypoints found in the processed image slices
		m_IsDetecting = true;
	}
	else
	{
		X = 0;
		Y = 0;
	}

	// Clear previously detected point
	m_DetectedPoints.clear();
	//m_DetectedPoints.empty();

	// Save the coordinates and the size of the blob
	// If we detected something
	if (X > .0 && Y > .0)
	{
		m_DetectedPoints.insert(make_pair("x", X));
		m_DetectedPoints.insert(make_pair("y", Y));
		m_DetectedPoints.insert(make_pair("size", size));
	}

	return m_DetectedPoints;
}

bool ImageProcessor::IsDetecting()
{
	return m_IsDetecting;
}

void ImageProcessor::DisplayImage(bool display, Mat& frame)
{
	if (display == true)
	{
		if (frame.empty() == false)
		{
			// Encode as jpg
			std::vector<unsigned char> buf;
			imencode(".jpg", frame, buf);

			int size = buf.size();
			boost::shared_array<char> data = boost::shared_array<char>(new char[size]);

			std::copy(buf.begin(), buf.end(), data.get());

			m_VideoPlayer->pushEncodedFrame(0, data, size);
		}
	}
}