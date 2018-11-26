//
// Created by yxt on 18-11-23.
//

#ifndef CAMERANODE_CAMERANODE_H
#define CAMERANODE_CAMERANODE_H

#include <iostream>
#include <sstream>
#include <string>
#include "Spinnaker.h"
#include "SpinGenApi/SpinnakerGenApi.h"

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

using namespace Spinnaker;
using namespace Spinnaker::GenApi;
using namespace Spinnaker::GenICam;
using namespace std;

int mcount = 0;
string mfilename="";
string path = "/home/yxt/test/camera/";

class ImageEventHandler : public ImageEvent
{
public:

	// The constructor retrieves the serial number and initializes the image
	// counter to 0.
	ImageEventHandler(CameraPtr pCam)
	{
		// Retrieve device serial number
		INodeMap & nodeMap = pCam->GetTLDeviceNodeMap();

		m_deviceSerialNumber = "";
		CStringPtr ptrDeviceSerialNumber = nodeMap.GetNode("DeviceSerialNumber");
		if (IsAvailable(ptrDeviceSerialNumber) && IsReadable(ptrDeviceSerialNumber))
		{
			m_deviceSerialNumber = ptrDeviceSerialNumber->GetValue();
		}

		// Initialize image counter to 0
		m_imageCnt = 0;

		// Release reference to camera
		pCam = NULL;
	}
	~ImageEventHandler() {}

	// This method defines an image event. In it, the image that triggered the
	// event is converted and saved before incrementing the count. Please see
	// Acquisition_CSharp example for more in-depth comments on the acquisition
	// of images.
	void OnImageEvent(ImagePtr image)
	{
		// Save a maximum of 10 images
		if (m_imageCnt < mk_numImages)
		{
			cout << "Image event occurred..." << endl;

			// Check image retrieval status
			if (image->IsIncomplete())
			{
				cout << "Image incomplete with image status " << image->GetImageStatus() << "..." << endl << endl;
			}
			else
			{
				// Print image information
				//cout << "Grabbed image " << m_imageCnt << ", width = " << image->GetWidth() << ", height = " << image->GetHeight() << endl;

				// Convert image to mono 8
				ImagePtr convertedImage = image->Convert(PixelFormat_Mono8, HQ_LINEAR);

				// Create a unique filename and save image
				ostringstream filename;

				filename << path;
				if (m_deviceSerialNumber != "")
				{
					filename << m_deviceSerialNumber.c_str() << "-";
				}
				//filename << m_imageCnt << ".jpg";
				filename << mcount << ".jpg";
				convertedImage->Save(filename.str().c_str());

				cout << "Image saved at " << filename.str() << endl;

				// Increment image counter
				m_imageCnt++;
			}
		}
	}

	// Getter for image counter
	int getImageCount()
	{
		return m_imageCnt;
	}

	// Getter for maximum images
	int getMaxImages()
	{
		return mk_numImages;
	}

private:

	static const unsigned int mk_numImages = 1;
	unsigned int m_imageCnt;
	string m_deviceSerialNumber;
};

class CameraNode
{
public:
	CameraNode();
	~CameraNode();
	void run();
	int RunSingleCamera(CameraPtr pCam);
	int AcquireImages(CameraPtr pCam, INodeMap & nodeMap, INodeMap & nodeMapTLDevice, ImageEventHandler*& imageEventHandler);
	int PrintDeviceInfo(INodeMap & nodeMap);
	int ResetImageEvents(CameraPtr pCam, ImageEventHandler*& imageEventHandler);
	int ConfigureImageEvents(CameraPtr pCam, ImageEventHandler*& imageEventHandler);
private:
	void Callback(const sensor_msgs::PointCloud2 &scan);
	int result;
	SystemPtr system;
	CameraList camList;
	unsigned int numCameras;
	ros::NodeHandle node_handle;
	ros::Subscriber camera_sub;
};


#endif //CAMERANODE_CAMERANODE_H
