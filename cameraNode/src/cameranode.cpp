//
// Created by yxt on 18-11-23.
//
#include "../include/cameranode.h"

using namespace Spinnaker;
using namespace Spinnaker::GenApi;
using namespace Spinnaker::GenICam;
using namespace std;


// This helper function allows the example to sleep in both Windows and Linux
// systems.
void SleepyWrapper(int milliseconds)
{
#if defined WIN32 || defined _WIN32 || defined WIN64 || defined _WIN64
	Sleep(milliseconds);
#else
	usleep(milliseconds);
#endif
}


CameraNode::CameraNode()
{
	ROS_INFO("Initialize...");
	result = 0;
	// Retrieve singleton reference to system object
	system = System::GetInstance();
	// Retrieve list of cameras from the system
	camList = system->GetCameras();
	numCameras = camList.GetSize();
	camera_sub = node_handle.subscribe("velodyne_points", 1028, &CameraNode::Callback, this);
	cout << "Number of cameras detected: " << numCameras << endl << endl;
}

CameraNode::~CameraNode()
{
	CameraPtr pCam = NULL;
	for (unsigned int i = 0; i < camList.GetSize(); i++)
	{
		// Select camera
		pCam = camList.GetByIndex(i);
		// Deinitialize camera
		pCam->DeInit();
	}
	// Clear camera list before releasing system
	camList.Clear();
	// Release system
	system->ReleaseInstance();
}


// This function configures the example to execute image events by preparing and
// registering an image event.
int CameraNode::ConfigureImageEvents(CameraPtr pCam, ImageEventHandler*& imageEventHandler)
{
	int result = 0;

	try
	{
		//
		// Create image event
		//
		// *** NOTES ***
		// The class has been constructed to accept a camera pointer in order
		// to allow the saving of images with the device serial number.
		//
		imageEventHandler = new ImageEventHandler(pCam);

		//
		// Register image event handler
		//
		// *** NOTES ***
		// Image events are registered to cameras. If there are multiple
		// cameras, each camera must have the image events registered to it
		// separately. Also, multiple image events may be registered to a
		// single camera.
		//
		// *** LATER ***
		// Image events must be unregistered manually. This must be done prior
		// to releasing the system and while the image events are still in
		// scope.
		//
		pCam->RegisterEvent(*imageEventHandler);
	}
	catch (Spinnaker::Exception &e)
	{
		cout << "Error: " << e.what() << endl;
		result = -1;
	}

	return result;
}

// This function waits for the appropriate amount of images.  Notice that
// whereas most examples actively retrieve images, the acquisition of images is
// handled passively in this example.
int WaitForImages(ImageEventHandler*& imageEventHandler)
{
	int result = 0;

	try
	{
		//
		// Wait for images
		//
		// *** NOTES ***
		// In order to passively capture images using image events and
		// automatic polling, the main thread sleeps in increments of 200 ms
		// until 10 images have been acquired and saved.
		//
		const int sleepDuration = 50; // in milliseconds

		while (imageEventHandler->getImageCount() < imageEventHandler->getMaxImages())
		{
			//cout << "\t//" << endl;
			//cout << "\t// Sleeping for " << sleepDuration << " ms. Grabbing images..." << endl;
			//cout << "\t//" << endl;
			SleepyWrapper(sleepDuration);
		}
	}
	catch (Spinnaker::Exception &e)
	{
		cout << "Error: " << e.what() << endl;
		result = -1;
	}

	return result;
}

// This functions resets the example by unregistering the image event.
int CameraNode::ResetImageEvents(CameraPtr pCam, ImageEventHandler*& imageEventHandler)
{
	int result = 0;

	try
	{
		//
		// Unregister image event handler
		//
		// *** NOTES ***
		// It is important to unregister all image events from all cameras
		// they are registered to.
		//
		pCam->UnregisterEvent(*imageEventHandler);

		// Delete image event (because it is a pointer)
		delete imageEventHandler;

		//cout << "Image events unregistered..." << endl << endl;
	}
	catch (Spinnaker::Exception &e)
	{
		cout << "Error: " << e.what() << endl;
		result = -1;
	}

	return result;
}

// This function prints the device information of the camera from the transport
// layer; please see NodeMapInfo example for more in-depth comments on printing
// device information from the nodemap.
int CameraNode::PrintDeviceInfo(INodeMap & nodeMap)
{
	int result = 0;

	cout << endl << "*** DEVICE INFORMATION ***" << endl << endl;

	try
	{
		FeatureList_t features;
		CCategoryPtr category = nodeMap.GetNode("DeviceInformation");
		if (IsAvailable(category) && IsReadable(category))
		{
			category->GetFeatures(features);

			FeatureList_t::const_iterator it;
			for (it = features.begin(); it != features.end(); ++it)
			{
				CNodePtr pfeatureNode = *it;
				cout << pfeatureNode->GetName() << " : ";
				CValuePtr pValue = (CValuePtr)pfeatureNode;
				cout << (IsReadable(pValue) ? pValue->ToString() : "Node not readable");
				cout << endl;
			}
		}
		else
		{
			cout << "Device control information not available." << endl;
		}
	}
	catch (Spinnaker::Exception &e)
	{
		cout << "Error: " << e.what() << endl;
		result = -1;
	}

	return result;
}


// This function passively waits for images by calling WaitForImages(). Notice that
// this function is much shorter than the AcquireImages() function of other examples.
// This is because most of the code has been moved to the image event's OnImageEvent()
// method.
int CameraNode::AcquireImages(CameraPtr pCam, INodeMap & nodeMap, INodeMap & nodeMapTLDevice, ImageEventHandler*& imageEventHandler)
{
	int result = 0;

	//cout << endl << endl << "*** IMAGE ACQUISITION ***" << endl << endl;

	try
	{
		// Set acquisition mode to continuous
		CEnumerationPtr ptrAcquisitionMode = nodeMap.GetNode("AcquisitionMode");
		if (!IsAvailable(ptrAcquisitionMode) || !IsWritable(ptrAcquisitionMode))
		{
			cout << "Unable to set acquisition mode to continuous (node retrieval). Aborting..." << endl << endl;
			return -1;
		}

		CEnumEntryPtr ptrAcquisitionModeContinuous = ptrAcquisitionMode->GetEntryByName("Continuous");
		if (!IsAvailable(ptrAcquisitionModeContinuous) || !IsReadable(ptrAcquisitionModeContinuous))
		{
			cout << "Unable to set acquisition mode to continuous (enum entry retrieval). Aborting..." << endl << endl;
			return -1;
		}

		int64_t acquisitionModeContinuous = ptrAcquisitionModeContinuous->GetValue();
		ptrAcquisitionMode->SetIntValue(acquisitionModeContinuous);
		//cout << "Acquisition mode set to continuous..." << endl;
		// Begin acquiring images
		pCam->BeginAcquisition();
		//cout << "Acquiring images..." << endl;
		// Retrieve images using image event handler
		WaitForImages(imageEventHandler);

		// End acquisition
		pCam->EndAcquisition();
	}
	catch (Spinnaker::Exception &e)
	{
		cout << "Error: " << e.what() << endl;
		result = -1;
	}

	return result;
}

// This function acts as the body of the example; please see NodeMapInfo example
// for more in-depth comments on setting up cameras.
int CameraNode::RunSingleCamera(CameraPtr pCam)
{
	int result = 0;
	int err = 0;

	try
	{
		// Retrieve TL device nodemap and print device information
		INodeMap & nodeMapTLDevice = pCam->GetTLDeviceNodeMap();
		//result = PrintDeviceInfo(nodeMapTLDevice);
		// Initialize camera
		//pCam->Init();
		// Retrieve GenICam nodemap
		INodeMap & nodeMap = pCam->GetNodeMap();
		// Configure image events
		ImageEventHandler* imageEventHandler;
		err = ConfigureImageEvents(pCam, imageEventHandler);
		if (err < 0)
		{
			return err;
		}
		// Acquire images using the image event handler
		result = result | AcquireImages(pCam, nodeMap, nodeMapTLDevice, imageEventHandler);
		// Reset image events
		result = result | ResetImageEvents(pCam, imageEventHandler);
		// Deinitialize camera
		//pCam->DeInit();
	}
	catch (Spinnaker::Exception &e)
	{
		cout << "Error: " << e.what() << endl;
		result = -1;
	}

	return result;
}

void CameraNode::Callback(const sensor_msgs::PointCloud2 &scan)
{
	stringstream time;
	time << ros::Time::now();
	time >> mfilename;
	// Run example on each camera
	for (unsigned int i = 0; i < numCameras; i++)
	{
		//cout << endl << "Running example for camera " << i << "..." << endl;
		result = result | this->RunSingleCamera(camList.GetByIndex(i));
		//cout << "Camera " << i << " example complete..." << endl << endl;
	}
	mcount++;
}

void CameraNode::run()
{
	CameraPtr pCam = NULL;

	// Finish if there are no cameras
	if (numCameras == 0)
	{
		// Clear camera list before releasing system
		camList.Clear();
		// Release system
		system->ReleaseInstance();
		cout << "Not enough cameras! Return." << endl;
		return;
	}

	// initialize
	for (unsigned int i = 0; i < camList.GetSize(); ++i) {
		pCam = camList.GetByIndex(i);
		pCam->Init();
	}
	ros::spin();

}

int main(int argc, char** argv)
{
	ros::init(argc,argv,"cameranode");
	CameraNode cameranode;
	cameranode.run();

	return 0;

}