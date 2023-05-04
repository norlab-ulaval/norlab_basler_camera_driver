#include <cmath>
#include <string>
#include <vector>
#include <map>
#include <boost/thread.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <chrono>

#include <ros/ros.h>
#include "std_msgs/String.h"
#include <std_msgs/UInt8.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <norlab_basler_camera_driver/packets_msg.h>
#include <norlab_basler_camera_driver/metadata_msg.h>

#include <pylon/PylonIncludes.h>
#include <pylon/BaslerUniversalCameraEventHandler.h>
#include <pylon/BaslerUniversalInstantCamera.h>
#include <pylon/BaslerUniversalInstantCameraArray.h>
#include <pylon/AcquireSingleFrameConfiguration.h>
#include <pylon/ImageDecompressor.h>

using namespace std;
using namespace Pylon;
using namespace cv;
using namespace Basler_UniversalCameraParams;

// Camera array
std::unique_ptr<CBaslerUniversalInstantCameraArray> cameras;
size_t maxCamerasToUse = 2;
int camera1_index;
int camera2_index;

// Params
std::map<string, string> parameters;
bool enable_bracketing;
bool enable_panoramic;
float gain;

// Camera Events
vector<string> Camera1FrameStartEventsFrameId;
vector<string> Camera1FrameStartEventsTimestamp;
vector<string> Camera1ExposureEndEventsFrameId;
vector<string> Camera1ExposureEndEventsTimestamp;

vector<string> Camera2FrameStartEventsFrameId;
vector<string> Camera2FrameStartEventsTimestamp;
vector<string> Camera2ExposureEndEventsFrameId;
vector<string> Camera2ExposureEndEventsTimestamp;

image_transport::CameraPublisher camera1_info_pub;
image_transport::CameraPublisher camera2_info_pub;
std::unique_ptr<camera_info_manager::CameraInfoManager> c1info_;
std::unique_ptr<camera_info_manager::CameraInfoManager> c2info_;
ros::Publisher camera1_packets_pub;
ros::Publisher camera2_packets_pub;
ros::Publisher camera1_metadata_pub;
ros::Publisher camera2_metadata_pub;

norlab_basler_camera_driver::packets_msg camera1_packets_msg;
norlab_basler_camera_driver::metadata_msg camera1_metadata_msg;
norlab_basler_camera_driver::packets_msg camera2_packets_msg;
norlab_basler_camera_driver::metadata_msg camera2_metadata_msg;

// Init
CImageDecompressor camera1_decompressor;
CImageDecompressor camera2_decompressor;
cv_bridge::CvImage camera_info_msg;
CBaslerUniversalGrabResultPtr camera1_ptrGrabResult;
CBaslerUniversalGrabResultPtr camera2_ptrGrabResult;

//Enumeration used for distinguishing different events.
enum MyEvents
{
    eMyExposureEndEvent = 100,
    eMyEventFrameStart = 200,
    eMyEventTemperatureStatusChangedStatus = 300,
};

// Example handler for camera events.
class CSampleCameraEventHandler : public CBaslerUniversalCameraEventHandler
{
public:
    // Only very short processing tasks should be performed by this method. Otherwise, the event notification will block the
    // processing of images.
    virtual void OnCameraEvent( CBaslerUniversalInstantCamera& camera, intptr_t userProvidedId, GenApi::INode* /* pNode */ )
    {
        if (camera.GetDeviceInfo().GetUserDefinedName() == "Camera_1")
        {
            switch (userProvidedId)
            {
            case eMyExposureEndEvent:
                if (camera.EventExposureEndFrameID.IsReadable()) // Applies to cameras based on SFNC 2.0 or later, e.g, USB cameras
                {
                    Camera1ExposureEndEventsFrameId.insert(Camera1ExposureEndEventsFrameId.begin(), to_string(camera.EventExposureEndFrameID.GetValue()));
                    Camera1ExposureEndEventsTimestamp.insert(Camera1ExposureEndEventsTimestamp.begin(), to_string(camera.EventExposureEndTimestamp.GetValue()));
                }
                break;
            case eMyEventFrameStart:
                Camera1FrameStartEventsFrameId.insert(Camera1FrameStartEventsFrameId.begin(), to_string(camera.EventFrameStartFrameID.GetValue()));
                Camera1FrameStartEventsTimestamp.insert(Camera1FrameStartEventsTimestamp.begin(), to_string(camera.EventFrameStartTimestamp.GetValue()));
                break;
            case eMyEventTemperatureStatusChangedStatus:
                ROS_INFO_STREAM("Camera1 Temperature Status Changed to: " << to_string(camera.EventTemperatureStatusChanged.GetValue()) << " at timestamp " << to_string(camera.EventTemperatureStatusChangedTimestamp.GetValue()) << endl);
            }
        }
        else
        {
            switch (userProvidedId)
            {
            case eMyExposureEndEvent:
                if (camera.EventExposureEndFrameID.IsReadable()) // Applies to cameras based on SFNC 2.0 or later, e.g, USB cameras
                {
                    Camera2ExposureEndEventsFrameId.insert(Camera2ExposureEndEventsFrameId.begin(), to_string(camera.EventExposureEndFrameID.GetValue()));
                    Camera2ExposureEndEventsTimestamp.insert(Camera2ExposureEndEventsTimestamp.begin(), to_string(camera.EventExposureEndTimestamp.GetValue()));
                }
                break;
            case eMyEventFrameStart:
                Camera2FrameStartEventsFrameId.insert(Camera2FrameStartEventsFrameId.begin(), to_string(camera.EventFrameStartFrameID.GetValue()));
                Camera2FrameStartEventsTimestamp.insert(Camera2FrameStartEventsTimestamp.begin(), to_string(camera.EventFrameStartTimestamp.GetValue()));
                break;
            case eMyEventTemperatureStatusChangedStatus:
                ROS_INFO_STREAM("Camera2 Temperature Status Changed to: " << to_string(camera.EventTemperatureStatusChanged.GetValue()) << " at timestamp " << to_string(camera.EventTemperatureStatusChangedTimestamp.GetValue()) << endl);
            }
        }
    }
};

void EnableMetadata(CBaslerUniversalInstantCamera& camera)
{
    if (!camera.ChunkModeActive.TrySetValue(true))
    {
        ROS_WARN("The camera does not support chunk features");
    }
    camera.ChunkSelector.SetValue(Basler_UniversalCameraParams::ChunkSelector_FrameID);
    camera.ChunkEnable.SetValue(true);
    camera.ChunkSelector.SetValue(Basler_UniversalCameraParams::ChunkSelector_Timestamp);
    camera.ChunkEnable.SetValue(true);
    camera.ChunkSelector.SetValue(Basler_UniversalCameraParams::ChunkSelector_ExposureTime);
    camera.ChunkEnable.SetValue(true);
}

void SetStartupUserSet(CBaslerUniversalInstantCamera& camera)
{
    if (parameters["startup_user_set"] == "UserSet1")
    {
        camera.UserSetSelector.SetValue(Basler_UniversalCameraParams::UserSetSelector_UserSet1);
        camera.UserSetLoad.Execute();
        cout << "Loaded: User Set 1" << endl;
    }
    else if (parameters["startup_user_set"] == "UserSet2")
    {
        camera.UserSetSelector.SetValue(Basler_UniversalCameraParams::UserSetSelector_UserSet2);
        camera.UserSetLoad.Execute();
        cout << "Loaded: User Set 2" << endl;
    }
    else if (parameters["startup_user_set"] == "UserSet3")
    {
        camera.UserSetSelector.SetValue(Basler_UniversalCameraParams::UserSetSelector_UserSet3);
        camera.UserSetLoad.Execute();
        cout << "Loaded: User Set 3" << endl;
    }

    if (parameters["image_encoding"] == "bayer_rggb12")
    {
        camera.PixelFormat.SetValue(Basler_UniversalCameraParams::PixelFormat_BayerRG12);
    }
    else if (parameters["image_encoding"] == "bayer_rggb8")
    {
        camera.PixelFormat.SetValue(Basler_UniversalCameraParams::PixelFormat_BayerRG8);
    }
}

void SetParameters(CBaslerUniversalInstantCamera& camera)
{
    camera.Gain.SetValue(gain);
}

void CreateAndOpenPylonDevice(CTlFactory& tlFactory, CDeviceInfo device, CBaslerUniversalInstantCamera& camera, size_t i)
{
    camera.Attach(tlFactory.CreateDevice(device));
    // Print the model name of the camera.
    cout << "Using device: " << camera.GetDeviceInfo().GetUserDefinedName() << endl;
    cout << "Device Index: " << i << endl;

    if (camera.GetDeviceInfo().GetUserDefinedName() == "Camera_1")
    {
        camera1_index = i;
        camera2_index = (camera1_index + 1) % 2;
    }
    camera.Open();
}

void SetEventsHandlers(CSampleCameraEventHandler* pHandler1, CBaslerUniversalInstantCamera& camera){
    camera.RegisterCameraEventHandler( pHandler1, "EventFrameStart", eMyEventFrameStart, RegistrationMode_ReplaceAll, Cleanup_None );
    camera.RegisterCameraEventHandler( pHandler1, "EventExposureEndData", eMyExposureEndEvent, RegistrationMode_Append, Cleanup_None );
    camera.RegisterCameraEventHandler( pHandler1, "EventTemperatureStatusChangedStatus", eMyEventTemperatureStatusChangedStatus, RegistrationMode_Append, Cleanup_None );

    // Enable sending of Exposure End events.
    // Select the event to receive.
    camera.EventSelector.SetValue( EventSelector_ExposureEnd );

    // Enable it.
    if (!camera.EventNotification.TrySetValue( EventNotification_On ))
    {
        cout << "Was not able to enable event Exposure End" << endl;
    }

    // Enable event notification for the FrameStart event, if available
    if (camera.EventSelector.TrySetValue( EventSelector_FrameStart ))
    {
        // Enable it.
        if (!camera.EventNotification.TrySetValue( EventNotification_On ))
        {
            cout << "Was not able to enable event Frame Start" << endl;
        }
    }

    // Enable event notification for the Temperature event, if available
    if (camera.EventSelector.TrySetValue( EventSelector_TemperatureStatusChanged ))
    {
        // Enable it.
        if (!camera.EventNotification.TrySetValue( EventNotification_On ))
        {
            cout << "Was not able to enable event Temperature Status Changed" << endl;
        }
    }
}

bool InitCameras()
{
    PylonInitialize();
    CTlFactory& tlFactory = CTlFactory::GetInstance();

    // Get all attached devices and exit application if no device is found.
    DeviceInfoList_t devices;
    if (tlFactory.EnumerateDevices(devices) == 0)
    {
        throw RUNTIME_EXCEPTION("No camera present.");
    }

    // Create an array of instant cameras for the found devices and avoid exceeding a maximum number of devices.
    cameras = std::unique_ptr<CBaslerUniversalInstantCameraArray>(new CBaslerUniversalInstantCameraArray(min(devices.size(), maxCamerasToUse)));
    
    // Create and attach all Pylon Devices.
    for (size_t i = 0; i < cameras->GetSize(); ++i)
    {
        (*cameras)[i].GrabCameraEvents = true;
        CreateAndOpenPylonDevice(tlFactory, devices[i], (*cameras)[i], i);
        SetStartupUserSet((*cameras)[i]);
        SetParameters((*cameras)[i]);
        EnableMetadata((*cameras)[i]); 
    }

    camera1_decompressor = CImageDecompressor((*cameras)[camera1_index].GetNodeMap());
    camera2_decompressor = CImageDecompressor((*cameras)[camera2_index].GetNodeMap());
    return true;
}

void StartGrabbing()
{
    cameras->StartGrabbing(GrabStrategy_LatestImageOnly);
}

void PublishCamInfoData(sensor_msgs::CameraInfo camera_info, string frame_id, image_transport::CameraPublisher& publisher, ros::Time time)
{
    camera_info_msg.header.stamp = time;
    camera_info_msg.header.frame_id = frame_id;
    sensor_msgs::CameraInfoPtr ci(new sensor_msgs::CameraInfo(camera_info));
    ci->header.frame_id = camera_info_msg.header.frame_id;
    ci->header.stamp = camera_info_msg.header.stamp;
    publisher.publish(camera_info_msg.toImageMsg(), ci);
}

void PublishCamMetadata(CBaslerUniversalGrabResultPtr image_ptr, norlab_basler_camera_driver::metadata_msg& msg, ros::Publisher& publisher, ros::Time time, vector<string>& FrameStartFrameId, vector<string>& FrameStartTimestamp, vector<string>& ExposureEndFrameId, vector<string>& ExposureEndTimestamp)
{
    msg.header.stamp = time;
    msg.FrameId = (int32_t)(image_ptr->ChunkFrameID.GetValue());
    msg.Timestamp = (int64_t)(image_ptr->ChunkTimestamp.GetValue());

    vector<string>::iterator itrFrameStart = find(FrameStartFrameId.begin(), FrameStartFrameId.end(), to_string(msg.FrameId));
    vector<string>::iterator itrExposureEnd = find(ExposureEndFrameId.begin(), ExposureEndFrameId.end(), to_string(msg.FrameId));

    float32_t ExposureTime = (stof(ExposureEndTimestamp[itrExposureEnd - ExposureEndFrameId.begin()]) - stof(FrameStartTimestamp[itrFrameStart - FrameStartFrameId.begin()]))*1e-6;
    msg.ExposureTime = ExposureTime;
    if(FrameStartFrameId.size() >= 3 && ExposureEndFrameId.size() >= 3){
        FrameStartFrameId.pop_back();
        FrameStartTimestamp.pop_back();
        ExposureEndFrameId.pop_back();
        ExposureEndTimestamp.pop_back();
    }
    publisher.publish(msg);
}

void PublishCamPackets(CImageDecompressor& camera_decompressor, CBaslerUniversalGrabResultPtr image_ptr, norlab_basler_camera_driver::packets_msg msg, ros::Publisher& publisher, ros::Time time)
{
    if(msg.descriptor_size == 0)
    {
        camera_decompressor.GetCompressionDescriptor(NULL, &msg.descriptor_size);
        msg.descriptor.resize(msg.descriptor_size);
    }
    msg.header.stamp = time;

    camera_decompressor.GetCompressionDescriptor(msg.descriptor.data(), &msg.descriptor_size);
    msg.imgSize = image_ptr->GetPayloadSize();
    msg.imgBuffer.resize(msg.imgSize);
    memcpy(msg.imgBuffer.data(), image_ptr->GetBuffer(), msg.imgSize);

    publisher.publish(msg);
}

void GrabLoop()
{
    (*cameras)[camera2_index].RetrieveResult(500, camera2_ptrGrabResult, TimeoutHandling_ThrowException);
    (*cameras)[camera1_index].RetrieveResult(500, camera1_ptrGrabResult, TimeoutHandling_ThrowException);

    if (camera1_ptrGrabResult->GrabSucceeded() && camera2_ptrGrabResult->GrabSucceeded())
    {
        ros::Time timestamp_ros = ros::Time::now();

        PublishCamInfoData(c1info_->getCameraInfo(), "camera1_link", camera1_info_pub, timestamp_ros);
        PublishCamInfoData(c2info_->getCameraInfo(), "camera2_link", camera2_info_pub, timestamp_ros);

        PublishCamPackets(camera1_decompressor, camera1_ptrGrabResult, camera1_packets_msg, camera1_packets_pub, timestamp_ros);
        PublishCamPackets(camera2_decompressor, camera2_ptrGrabResult, camera2_packets_msg, camera2_packets_pub, timestamp_ros);
        PublishCamMetadata(camera1_ptrGrabResult, camera1_metadata_msg, camera1_metadata_pub, timestamp_ros, Camera1FrameStartEventsFrameId, Camera1FrameStartEventsTimestamp, Camera1ExposureEndEventsFrameId, Camera1ExposureEndEventsTimestamp);
        PublishCamMetadata(camera2_ptrGrabResult, camera2_metadata_msg, camera2_metadata_pub, timestamp_ros, Camera2FrameStartEventsFrameId, Camera2FrameStartEventsTimestamp, Camera2ExposureEndEventsFrameId, Camera2ExposureEndEventsTimestamp);

        camera1_ptrGrabResult.Release();
        camera2_ptrGrabResult.Release();
    }
    else
    {
        ROS_INFO_STREAM("Error Camera1: " << std::hex << camera1_ptrGrabResult->GetErrorCode() << std::dec << " " << camera1_ptrGrabResult->GetErrorDescription() << endl);
        ROS_INFO_STREAM("Error Camera2: " << std::hex << camera2_ptrGrabResult->GetErrorCode() << std::dec << " " << camera2_ptrGrabResult->GetErrorDescription() << endl);
    }
}

void GetParameters(ros::NodeHandle handler)
{
    handler.getParam("/stereo/norlab_basler_camera_driver_node/startup_user_set", parameters["startup_user_set"]);
    handler.getParam("/stereo/norlab_basler_camera_driver_node/image_encoding", parameters["image_encoding"]);
    handler.getParam("/stereo/norlab_basler_camera_driver_node/camera1_calibration_url", parameters["camera1_calibration_url"]);
    handler.getParam("/stereo/norlab_basler_camera_driver_node/camera2_calibration_url", parameters["camera2_calibration_url"]);
    handler.getParam("/stereo/norlab_basler_camera_driver_node/gain", gain);
    handler.getParam("/stereo/norlab_basler_camera_driver_node/enable_bracketing", enable_bracketing);
    handler.getParam("/stereo/norlab_basler_camera_driver_node/enable_panoramic", enable_panoramic);
}

void InitCameraInfo(ros::NodeHandle cam1, ros::NodeHandle cam2)
{
    c1info_ = std::unique_ptr<camera_info_manager::CameraInfoManager>(new camera_info_manager::CameraInfoManager(cam1));
    c1info_->setCameraName((std::string)(*cameras)[camera1_index].GetDeviceInfo().GetUserDefinedName());
    c1info_->loadCameraInfo(parameters["camera1_calibration_url"]);
    c2info_ = std::unique_ptr<camera_info_manager::CameraInfoManager>(new camera_info_manager::CameraInfoManager(cam2));
    c2info_->setCameraName((std::string)(*cameras)[camera2_index].GetDeviceInfo().GetUserDefinedName());
    c2info_->loadCameraInfo(parameters["camera2_calibration_url"]);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "norlab_basler_camera_driver_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_cam1("camera1");
    ros::NodeHandle nh_cam2("camera2");
    // ros::NodeHandle nh_pano("panoramic_8bits");

    GetParameters(nh);
    ros::Rate r(50);
    camera_info_msg.image = cv::Mat();
    if (parameters["image_encoding"] == "bayer_rggb12")
    {
        camera_info_msg.encoding = sensor_msgs::image_encodings::BAYER_RGGB16;
    }
    else if (parameters["image_encoding"] == "bayer_rggb8")
    {
        camera_info_msg.encoding = sensor_msgs::image_encodings::BAYER_RGGB8;
    }
    
    image_transport::ImageTransport it_cam1(nh_cam1);
    image_transport::ImageTransport it_cam2(nh_cam2);
    camera1_info_pub = it_cam1.advertiseCamera("empty_image", 10);
    camera2_info_pub = it_cam2.advertiseCamera("empty_image", 10);

    camera1_packets_pub = nh.advertise<norlab_basler_camera_driver::packets_msg>("camera1/image_compressed", 10);
    camera2_packets_pub = nh.advertise<norlab_basler_camera_driver::packets_msg>("camera2/image_compressed", 10);
    camera1_metadata_pub = nh.advertise<norlab_basler_camera_driver::metadata_msg>("camera1/metadata", 10);
    camera2_metadata_pub = nh.advertise<norlab_basler_camera_driver::metadata_msg>("camera2/metadata", 10);

    InitCameras();
    CSampleCameraEventHandler* pHandler1 = new CSampleCameraEventHandler;
    SetEventsHandlers(pHandler1, (*cameras)[camera1_index]);
    SetEventsHandlers(pHandler1, (*cameras)[camera2_index]);
    InitCameraInfo(nh_cam1, nh_cam2);
    StartGrabbing();

    while ( ros::ok() )
    { 
        GrabLoop();
        r.sleep();
    }

    delete pHandler1;
    ROS_INFO("Terminate Stereo Bracketing Node");
    return EXIT_SUCCESS;
}
