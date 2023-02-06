#include <cmath>
#include <string>
#include <vector>
#include <map>
#include <boost/thread.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>

#include <pylon/PylonIncludes.h>
#include <pylon/BaslerUniversalInstantCamera.h>
#include <pylon/BaslerUniversalInstantCameraArray.h>
#include <pylon/AcquireSingleFrameConfiguration.h>

using namespace std;
using namespace Pylon;
using namespace cv;

std::unique_ptr<CBaslerUniversalInstantCameraArray> cameras;
std::map<string, string> parameters;
bool enable_bracketing;
int frame_rate;
image_transport::Publisher out_image_panoramic_pub;
image_transport::CameraPublisher out_image_camera1_pub;
image_transport::CameraPublisher out_image_camera2_pub;
std::unique_ptr<camera_info_manager::CameraInfoManager> c1info_;
std::unique_ptr<camera_info_manager::CameraInfoManager> c2info_;
size_t maxCamerasToUse = 2;
int camera1_index;
int camera2_index;
std::vector<float> exposures;
int idxExposures = 0;

// Example handler for camera events.
class CSampleImageEventHandler : public CImageEventHandler
{
public:
    virtual void OnImageGrabbed( CInstantCamera& /*camera*/, const CGrabResultPtr& /*ptrGrabResult*/ )
    {
        if (enable_bracketing)
        {
            (*cameras)[camera1_index].ExposureTime.SetValue(exposures[idxExposures]);
            (*cameras)[camera2_index].ExposureTime.SetValue(exposures[idxExposures]);
            idxExposures = (idxExposures + 1) % exposures.size();
        }
    }
};

void SetStartupUserSet(CBaslerUniversalInstantCamera& camera)
{
    if (parameters["startup_user_set"] == "UserSet1")
    {
        camera.UserSetSelector.SetValue(Basler_UniversalCameraParams::UserSetSelector_UserSet1);
        camera.UserSetLoad.Execute();
    }
}

void EnableChunkData(CBaslerUniversalInstantCamera& camera)
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
    camera.ChunkSelector.SetValue(Basler_UniversalCameraParams::ChunkSelector_PayloadCRC16);
    camera.ChunkEnable.SetValue(true);
}

void InitializeExposureTime()
{
    if (enable_bracketing)
    {
        (*cameras)[camera1_index].ExposureTime.SetValue(exposures[idxExposures]);        
        (*cameras)[camera2_index].ExposureTime.SetValue(exposures[idxExposures]);
        idxExposures = 1;
    }
    else
    {
        (*cameras)[camera1_index].ExposureAuto.SetValue(Basler_UniversalCameraParams::ExposureAuto_Continuous);        
        (*cameras)[camera2_index].ExposureAuto.SetValue(Basler_UniversalCameraParams::ExposureAuto_Continuous);
    }
}

void CreateAndOpenPylonDevice(CTlFactory& tlFactory, CDeviceInfo device, CBaslerUniversalInstantCamera& camera, size_t i)
{
    camera.Attach(tlFactory.CreateDevice(device));
    // Print the model name of the camera.
    cout << "Using device " << camera.GetDeviceInfo().GetUserDefinedName() << endl;
    cout << "Device Index " << i << endl;

    if (camera.GetDeviceInfo().GetUserDefinedName() == "Camera_1")
    {
        camera1_index = i;
        camera2_index = (camera1_index + 1) % 2;
        camera.RegisterImageEventHandler(new CSampleImageEventHandler, RegistrationMode_Append, Cleanup_Delete);
        camera.GrabCameraEvents = true;
    }
    camera.Open();
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
        CreateAndOpenPylonDevice(tlFactory, devices[i], (*cameras)[i], i);
        SetStartupUserSet((*cameras)[i]);
        EnableChunkData((*cameras)[i]);      
    }

    CAcquireSingleFrameConfiguration sf_config = CAcquireSingleFrameConfiguration();
    sf_config.ApplyConfiguration((*cameras)[camera1_index].GetNodeMap());

    InitializeExposureTime();
    return true;
}

void SetupPTP()
{
    // ** Configure PTP **
    // Set Priority 1 to 128
    (*cameras)[camera1_index].BslPtpPriority1.SetValue(1);
    (*cameras)[camera2_index].BslPtpPriority1.SetValue(128);
    // Enable end-to-end delay measurement
    (*cameras)[camera1_index].BslPtpProfile.SetValue(Basler_UniversalCameraParams::BslPtpProfile_DelayRequestResponseDefaultProfile);
    (*cameras)[camera2_index].BslPtpProfile.SetValue(Basler_UniversalCameraParams::BslPtpProfile_DelayRequestResponseDefaultProfile);
    // Set the network mode to unicast
    (*cameras)[camera1_index].BslPtpNetworkMode.SetValue(Basler_UniversalCameraParams::BslPtpNetworkMode_Unicast);
    (*cameras)[camera2_index].BslPtpNetworkMode.SetValue(Basler_UniversalCameraParams::BslPtpNetworkMode_Unicast);
    // Set the IP address of the first unicast device to 192.168.10.12
    // (0xC0 = 192, 0xA8 = 168, 0x0A = 10, 0x0C = 12)
    (*cameras)[camera1_index].BslPtpUcPortAddrIndex.SetValue(0);
    (*cameras)[camera1_index].BslPtpUcPortAddr.SetValue((0xC0A80303));
    (*cameras)[camera2_index].BslPtpUcPortAddrIndex.SetValue(1);
    (*cameras)[camera2_index].BslPtpUcPortAddr.SetValue((0xC0A80304));
    // Enable PTP Management Protocol
    (*cameras)[camera1_index].BslPtpManagementEnable.SetValue(true);
    (*cameras)[camera2_index].BslPtpManagementEnable.SetValue(true);
    // Disable two-step operation
    (*cameras)[camera1_index].BslPtpTwoStep.SetValue(false);
    (*cameras)[camera2_index].BslPtpTwoStep.SetValue(false);
    // ** Enable PTP on the current device **
    (*cameras)[camera1_index].PtpEnable.SetValue(true);
    (*cameras)[camera2_index].PtpEnable.SetValue(true);
    // To check the status of the PTP clock synchronization,
    // implement your own check method here.
    // For guidelines, see "Checking the Status of
    // the PTP Clock Synchronization" in this topic.
}

void StartGrabbing()
{
    cameras->StartGrabbing(GrabStrategy_LatestImageOnly); //GrabStrategy_LatestImageOnly
}

void DisplayDataOnImage(Mat& image, CBaslerUniversalGrabResultPtr ptrGrabResult)
{
    Point org_timestamp(30,100);
    Point org_frameID(30,200);
    Point org_exposuretime(30,300);
    cv::putText(image, std::to_string(ptrGrabResult->ChunkTimestamp.GetValue()), org_timestamp, FONT_HERSHEY_SCRIPT_COMPLEX, 2.1,
                    Scalar(0, 0, 255), 2, LINE_AA);
    cv::putText(image, std::to_string(ptrGrabResult->ChunkFrameID.GetValue()), org_frameID, FONT_HERSHEY_SCRIPT_COMPLEX, 2.1,
                    Scalar(0, 0, 255), 2, LINE_AA);
    cv::putText(image, std::to_string(ptrGrabResult->ChunkExposureTime.GetValue()), org_exposuretime, FONT_HERSHEY_SCRIPT_COMPLEX, 2.1,
                    Scalar(0, 0, 255), 2, LINE_AA);
}

void PublishCamData(Mat& image, sensor_msgs::CameraInfo camera_info, string frame_id, image_transport::CameraPublisher& publisher)
{
    std_msgs::Header camera_header;
    camera_header.frame_id = frame_id;
    sensor_msgs::ImagePtr out_image_camera = cv_bridge::CvImage(camera_header, parameters["image_encoding"], image).toImageMsg();
    sensor_msgs::CameraInfoPtr ci(new sensor_msgs::CameraInfo(camera_info));
    ci->header.frame_id = out_image_camera->header.frame_id;
    ci->header.stamp = out_image_camera->header.stamp;
    publisher.publish(out_image_camera, ci);
}

void GrabLoop()
{
    CBaslerUniversalGrabResultPtr camera1_ptrGrabResult;
    CBaslerUniversalGrabResultPtr camera2_ptrGrabResult;
    (*cameras)[camera1_index].AcquisitionStart.Execute();
    (*cameras)[camera2_index].RetrieveResult(500, camera2_ptrGrabResult, TimeoutHandling_ThrowException);
    (*cameras)[camera1_index].RetrieveResult(500, camera1_ptrGrabResult, TimeoutHandling_ThrowException);
    if (camera1_ptrGrabResult->GrabSucceeded() && camera2_ptrGrabResult->GrabSucceeded())
    {
        Mat image1 = Mat(camera1_ptrGrabResult->GetHeight(), camera1_ptrGrabResult->GetWidth(), CV_8UC1, (uint8_t *) camera1_ptrGrabResult->GetBuffer());
        Mat image2 = Mat(camera2_ptrGrabResult->GetHeight(), camera2_ptrGrabResult->GetWidth(), CV_8UC1, (uint8_t *) camera2_ptrGrabResult->GetBuffer());

        // DisplayDataOnImage(image1, camera1_ptrGrabResult);
        // DisplayDataOnImage(image2, camera2_ptrGrabResult);

        PublishCamData(image1, c1info_->getCameraInfo(), "camera1_link", out_image_camera1_pub);
        PublishCamData(image2, c2info_->getCameraInfo(), "camera2_link", out_image_camera2_pub);

        // Publish Panoramic
        Mat panoramic;
        try
        {
            hconcat(image1, image2, panoramic);
        }
        catch (...)
        {
            ROS_INFO("No concatenation");
        }
        std_msgs::Header panoramic_header;
        sensor_msgs::ImagePtr out_image_panoramic = cv_bridge::CvImage(panoramic_header, parameters["image_encoding"], panoramic).toImageMsg();
        out_image_panoramic_pub.publish(out_image_panoramic);
    }
    else
    {
        cout << "Error: " << std::hex << camera1_ptrGrabResult->GetErrorCode() << std::dec << " " << camera1_ptrGrabResult->GetErrorDescription() << endl;
    }
}

void GetParameters(ros::NodeHandle handler)
{
    handler.getParam("/stereo/norlab_basler_camera_driver_node/startup_user_set", parameters["startup_user_set"]); // Not Use for now
    handler.getParam("/stereo/norlab_basler_camera_driver_node/image_encoding", parameters["image_encoding"]);
    handler.getParam("/stereo/norlab_basler_camera_driver_node/camera1_calibration_url", parameters["camera1_calibration_url"]);
    handler.getParam("/stereo/norlab_basler_camera_driver_node/camera2_calibration_url", parameters["camera2_calibration_url"]);
    handler.getParam("/stereo/norlab_basler_camera_driver_node/frame_rate", frame_rate);
    handler.getParam("/stereo/norlab_basler_camera_driver_node/enable_bracketing", enable_bracketing);
    handler.getParam("/stereo/norlab_basler_camera_driver_node/bracketing_values", exposures);
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
    ros::NodeHandle nh_pano("panoramic");
    GetParameters(nh);
    ros::Rate r(frame_rate);

    image_transport::ImageTransport it_cam1(nh_cam1);
    image_transport::ImageTransport it_cam2(nh_cam2);
    image_transport::ImageTransport it_pano(nh_pano);
    out_image_panoramic_pub = it_pano.advertise("image", 10);
    out_image_camera1_pub = it_cam1.advertiseCamera("image", 10);
    out_image_camera2_pub = it_cam2.advertiseCamera("image", 10);

    InitCameras();
    InitCameraInfo(nh_cam1, nh_cam2);
    StartGrabbing();

    while ( ros::ok() )
    { 
        GrabLoop();
        r.sleep();
    }

    ROS_INFO("Terminate Stereo Bracketing Node");
    return EXIT_SUCCESS;
}
