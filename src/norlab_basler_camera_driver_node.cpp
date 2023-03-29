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
#include <norlab_basler_camera_driver/metadata_msg.h>

#include <pylon/PylonIncludes.h>
#include <pylon/BaslerUniversalInstantCamera.h>
#include <pylon/BaslerUniversalInstantCameraArray.h>
#include <pylon/AcquireSingleFrameConfiguration.h>
#include <pylon/ImageDecompressor.h>

using namespace std;
using namespace Pylon;
using namespace cv;

std::unique_ptr<CBaslerUniversalInstantCameraArray> cameras;
std::map<string, string> parameters;
bool enable_bracketing;
bool enable_panoramic;
float frame_rate;
image_transport::Publisher out_image_panoramic_RGB8_pub;
image_transport::CameraPublisher out_image_camera1_pub;
ros::Publisher metadata_pub;
image_transport::CameraPublisher out_image_camera2_pub;
std::unique_ptr<camera_info_manager::CameraInfoManager> c1info_;
std::unique_ptr<camera_info_manager::CameraInfoManager> c2info_;
size_t maxCamerasToUse = 2;
int camera1_index;
int camera2_index;
std::vector<float> exposures;
int idxExposures = 0;

norlab_basler_camera_driver::metadata_msg msg;
CImageDecompressor camera1_decompressor;
CImageDecompressor camera2_decompressor;
CompressionInfo_t cam_info;
CPylonImage camera1_targetImage;
CPylonImage camera2_targetImage;

CBaslerUniversalGrabResultPtr camera1_ptrGrabResult;
CBaslerUniversalGrabResultPtr camera2_ptrGrabResult;


class CEventHandler : public CBaslerUniversalCameraEventHandler, public CImageEventHandler
{
public:
    virtual void OnImageGrabbed( CInstantCamera& /*camera*/, const CGrabResultPtr& /*ptrGrabResult*/ )
    {
        //(*cameras)[camera1_index].AcquisitionStart.Execute();
        //(*cameras)[camera1_index].SoftwareSignalPulse.Execute();
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
    // camera.ChunkSelector.SetValue(Basler_UniversalCameraParams::ChunkSelector_SequencerSetActive);
    // camera.ChunkEnable.SetValue(true);
}

void SetSequenceExposure(CBaslerUniversalInstantCamera& camera)
{
    camera.SequencerMode.SetValue(Basler_UniversalCameraParams::SequencerMode_Off);
    camera.SequencerConfigurationMode.SetValue(Basler_UniversalCameraParams::SequencerConfigurationMode_On);

    for (int i = 0; i < exposures.size(); i++)
    {
        camera.SequencerSetSelector.SetValue(i);
        camera.SequencerSetLoad.Execute();
        camera.ExposureTime.SetValue(exposures[i]);
        camera.Gain.SetValue(1.0);
        if (parameters["image_encoding"] == "bayer_rggb12")
        {
            camera.PixelFormat.SetValue(Basler_UniversalCameraParams::PixelFormat_BayerRG12);
        }
        else if (parameters["image_encoding"] == "bayer_rggb8")
        {
            camera.PixelFormat.SetValue(Basler_UniversalCameraParams::PixelFormat_BayerRG8);
        }
        camera.SequencerSetStart.SetValue(0);
        camera.SequencerPathSelector.SetValue(0);
        camera.SequencerSetNext.SetValue((i+1)%exposures.size());
        camera.SequencerTriggerSource.SetValue("ExposureStart");
        camera.SequencerSetSave.Execute();
    }   

    camera.SequencerConfigurationMode.SetValue(Basler_UniversalCameraParams::SequencerConfigurationMode_Off);
    camera.SequencerMode.SetValue(Basler_UniversalCameraParams::SequencerMode_On);
}

void SetStartupUserSet(CBaslerUniversalInstantCamera& camera)
{
    if (parameters["startup_user_set"] == "UserSet1")
    {
        camera.SequencerMode.SetValue(Basler_UniversalCameraParams::SequencerMode_Off);
        camera.SequencerConfigurationMode.SetValue(Basler_UniversalCameraParams::SequencerConfigurationMode_Off);
        camera.UserSetSelector.SetValue(Basler_UniversalCameraParams::UserSetSelector_UserSet1);
        camera.UserSetLoad.Execute();
        if (parameters["image_encoding"] == "bayer_rggb12")
        {
            camera.PixelFormat.SetValue(Basler_UniversalCameraParams::PixelFormat_BayerRG12);
        }
        else if (parameters["image_encoding"] == "bayer_rggb8")
        {
            camera.PixelFormat.SetValue(Basler_UniversalCameraParams::PixelFormat_BayerRG8);
        }
    }
    else if (parameters["startup_user_set"] == "UserSet2")
    {
        camera.UserSetSelector.SetValue(Basler_UniversalCameraParams::UserSetSelector_UserSet2);
        camera.UserSetLoad.Execute();
        if (parameters["image_encoding"] == "bayer_rggb12")
        {
            camera.PixelFormat.SetValue(Basler_UniversalCameraParams::PixelFormat_BayerRG12);
        }
        else if (parameters["image_encoding"] == "bayer_rggb8")
        {
            camera.PixelFormat.SetValue(Basler_UniversalCameraParams::PixelFormat_BayerRG8);
        }
    }
}

void InitializeExposureTimeAuto()
{
    ROS_WARN("No bracketing. Using auto exposure time. Might decrease the frame rate.");
    (*cameras)[camera1_index].ExposureAuto.SetValue(Basler_UniversalCameraParams::ExposureAuto_Continuous);
    (*cameras)[camera2_index].ExposureAuto.SetValue(Basler_UniversalCameraParams::ExposureAuto_Continuous);
    // (*cameras)[camera1_index].ExposureTime.SetValue(16000);
    // (*cameras)[camera2_index].ExposureTime.SetValue(16000);
    if (parameters["image_encoding"] == "bayer_rggb12")
    {
        (*cameras)[camera1_index].PixelFormat.SetValue(Basler_UniversalCameraParams::PixelFormat_BayerRG12);        
        (*cameras)[camera2_index].PixelFormat.SetValue(Basler_UniversalCameraParams::PixelFormat_BayerRG12);
    }
    else if (parameters["image_encoding"] == "bayer_rggb8")
    {
        (*cameras)[camera1_index].PixelFormat.SetValue(Basler_UniversalCameraParams::PixelFormat_BayerRG8);        
        (*cameras)[camera2_index].PixelFormat.SetValue(Basler_UniversalCameraParams::PixelFormat_BayerRG8);
    }
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
        //camera.RegisterImageEventHandler(new CEventHandler(), RegistrationMode_Append, Cleanup_Delete);
        //camera.GrabCameraEvents = true;
    }
    camera.Open();
}

void setAcquisitionFrameRate(CBaslerUniversalInstantCamera& camera)
{
    camera.AcquisitionFrameRate.SetValue(frame_rate);
    camera.AcquisitionFrameRateEnable.SetValue(true);
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
        EnableMetadata((*cameras)[i]);   
    }

    camera1_decompressor = CImageDecompressor((*cameras)[camera1_index].GetNodeMap());
    camera2_decompressor = CImageDecompressor((*cameras)[camera2_index].GetNodeMap());

    //CAcquireSingleFrameConfiguration sf_config = CAcquireSingleFrameConfiguration();
    //sf_config.ApplyConfiguration((*cameras)[camera1_index].GetNodeMap());
    // setAcquisitionFrameRate((*cameras)[camera1_index]);
    // if (enable_bracketing)
    // {
    //     SetSequenceExposure((*cameras)[camera1_index]);
    // }
    // else
    // {
    //     InitializeExposureTimeAuto();
    // }
    return true;
}

void StartGrabbing()
{
    cameras->StartGrabbing(GrabStrategy_LatestImageOnly);
}

void PublishCamData(Mat& image, sensor_msgs::CameraInfo camera_info, string frame_id, image_transport::CameraPublisher& publisher, ros::Time time)
{
    cv_bridge::CvImage out_image_msg;
    out_image_msg.header.stamp = time;
    out_image_msg.header.frame_id = frame_id;
    if (parameters["image_encoding"] == "bayer_rggb12")
    {
        out_image_msg.encoding = sensor_msgs::image_encodings::BAYER_RGGB16;
    }
    else if (parameters["image_encoding"] == "bayer_rggb8")
    {
        out_image_msg.encoding = sensor_msgs::image_encodings::BAYER_RGGB8;
    }
    out_image_msg.image = image;
    sensor_msgs::CameraInfoPtr ci(new sensor_msgs::CameraInfo(camera_info));
    ci->header.frame_id = out_image_msg.header.frame_id;
    ci->header.stamp = out_image_msg.header.stamp;
    publisher.publish(out_image_msg.toImageMsg(), ci);
}

void PublishCamMetadata(CBaslerUniversalGrabResultPtr image1_ptr, CBaslerUniversalGrabResultPtr image2_ptr, ros::Publisher& publisher, ros::Time time)
{
    if(msg.size_1 == 0)
    {
        camera1_decompressor.GetCompressionDescriptor(NULL, &msg.size_1);
        camera2_decompressor.GetCompressionDescriptor(NULL, &msg.size_2);
        msg.descriptor_1.resize(msg.size_1);
        msg.descriptor_2.resize(msg.size_2);
    }
    msg.header.stamp = time;
    msg.imgFrameId = (int32_t)(image1_ptr->ChunkFrameID.GetValue());
    msg.exposureTime = (float32_t)(image1_ptr->ChunkExposureTime.GetValue());
    camera1_decompressor.GetCompressionDescriptor(msg.descriptor_1.data(), &msg.size_1);
    msg.imgSize_1 = image1_ptr->GetPayloadSize();
    msg.imgBuffer_1.resize(msg.imgSize_1);
    memcpy(msg.imgBuffer_1.data(), image1_ptr->GetBuffer(), msg.imgSize_1);

    camera2_decompressor.GetCompressionDescriptor(msg.descriptor_2.data(), &msg.size_2);
    msg.imgSize_2 = image2_ptr->GetPayloadSize();
    msg.imgBuffer_2.resize(msg.imgSize_2);
    memcpy(msg.imgBuffer_2.data(), image2_ptr->GetBuffer(), msg.imgSize_2);

    publisher.publish(msg);
}

void GrabLoop()
{
    // auto time_0 = std::chrono::system_clock::now();
    (*cameras)[camera2_index].RetrieveResult(500, camera2_ptrGrabResult, TimeoutHandling_ThrowException);
    (*cameras)[camera1_index].RetrieveResult(500, camera1_ptrGrabResult, TimeoutHandling_ThrowException);

    if (camera1_ptrGrabResult->GrabSucceeded() && camera2_ptrGrabResult->GrabSucceeded())
    {
        Mat cv_image1_bayerRG;
        Mat cv_image2_bayerRG;
        ros::Time timestamp_ros = ros::Time::now();

        if (parameters["image_encoding"] == "bayer_rggb12")
        {
            cv_image1_bayerRG = Mat(camera1_targetImage.GetHeight(), camera1_targetImage.GetWidth(), CV_16UC1, (uint16_t *) camera1_targetImage.GetBuffer());
            cv_image2_bayerRG = Mat(camera2_targetImage.GetHeight(), camera2_targetImage.GetWidth(), CV_16UC1, (uint16_t *) camera2_targetImage.GetBuffer());
        }
        else if (parameters["image_encoding"] == "bayer_rggb8")
        {
            cv_image1_bayerRG = Mat(camera1_targetImage.GetHeight(), camera1_targetImage.GetWidth(), CV_8UC1, (uint8_t *) camera1_targetImage.GetBuffer());
            cv_image2_bayerRG = Mat(camera2_targetImage.GetHeight(), camera2_targetImage.GetWidth(), CV_8UC1, (uint8_t *) camera2_targetImage.GetBuffer());
        }

        PublishCamData(cv_image1_bayerRG, c1info_->getCameraInfo(), "camera1_link", out_image_camera1_pub, timestamp_ros);
        PublishCamData(cv_image2_bayerRG, c2info_->getCameraInfo(), "camera2_link", out_image_camera2_pub, timestamp_ros);
        PublishCamMetadata(camera1_ptrGrabResult, camera2_ptrGrabResult, metadata_pub, timestamp_ros);

        // Publish panoramic 8bits images
        if (enable_panoramic && parameters["image_encoding"] == "bayer_rggb12")
        {
            Mat cv_image1_RGB16(cv_image1_bayerRG.cols, cv_image1_bayerRG.rows, CV_16UC3);
            cvtColor(cv_image1_bayerRG, cv_image1_RGB16, COLOR_BayerRG2RGB);
            Mat cv_image1_RGB8;
            cv_image1_RGB16.convertTo(cv_image1_RGB8, CV_8UC3, 1/16.0);

            Mat cv_image2_RGB16(cv_image2_bayerRG.cols, cv_image2_bayerRG.rows, CV_16UC3);
            cvtColor(cv_image2_bayerRG, cv_image2_RGB16, COLOR_BayerRG2RGB);
            Mat cv_image2_RGB8;
            cv_image2_RGB16.convertTo(cv_image2_RGB8, CV_8UC3, 1/16.0);
            Mat cv_panoramic_RGB8;
            try
            {
                hconcat(cv_image1_RGB8, cv_image2_RGB8, cv_panoramic_RGB8);
            }
            catch (...)
            {
                ROS_INFO("No concatenation");
            }
            cv_bridge::CvImage out_panoramic_RGB8_msg;
            out_panoramic_RGB8_msg.header.stamp = timestamp_ros;
            out_panoramic_RGB8_msg.header.frame_id = "panoramic_link";
            out_panoramic_RGB8_msg.encoding = sensor_msgs::image_encodings::TYPE_8UC3;
            out_panoramic_RGB8_msg.image = cv_panoramic_RGB8;
            out_image_panoramic_RGB8_pub.publish(out_panoramic_RGB8_msg.toImageMsg());
        }

        camera1_ptrGrabResult.Release();
        camera2_ptrGrabResult.Release();
    }
    else
    {
        ROS_INFO_STREAM("Error Camera1: " << std::hex << camera1_ptrGrabResult->GetErrorCode() << std::dec << " " << camera1_ptrGrabResult->GetErrorDescription() << endl);
        ROS_INFO_STREAM("Error Camera2: " << std::hex << camera2_ptrGrabResult->GetErrorCode() << std::dec << " " << camera2_ptrGrabResult->GetErrorDescription() << endl);
    }
    // auto time_4 = std::chrono::system_clock::now();
    // std::chrono::duration<double> elapsed_seconds_40 = time_4-time_0;
    // cout << "elapsed time 40: " << elapsed_seconds_40.count() << "s" << endl;
}

void GetParameters(ros::NodeHandle handler)
{
    handler.getParam("/stereo/norlab_basler_camera_driver_node/startup_user_set", parameters["startup_user_set"]);
    handler.getParam("/stereo/norlab_basler_camera_driver_node/image_encoding", parameters["image_encoding"]);
    handler.getParam("/stereo/norlab_basler_camera_driver_node/camera1_calibration_url", parameters["camera1_calibration_url"]);
    handler.getParam("/stereo/norlab_basler_camera_driver_node/camera2_calibration_url", parameters["camera2_calibration_url"]);
    handler.getParam("/stm32_node/frame_rate", frame_rate);
    handler.getParam("/stereo/norlab_basler_camera_driver_node/enable_bracketing", enable_bracketing);
    handler.getParam("/stereo/norlab_basler_camera_driver_node/enable_panoramic", enable_panoramic);
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
    ros::NodeHandle nh_pano("panoramic_8bits");
    GetParameters(nh);
    ros::Rate r(frame_rate);
    
    image_transport::ImageTransport it_cam1(nh_cam1);
    image_transport::ImageTransport it_cam2(nh_cam2);
    image_transport::ImageTransport it_pano(nh_pano);
    out_image_panoramic_RGB8_pub = it_pano.advertise("image", 10);
    out_image_camera1_pub = it_cam1.advertiseCamera("image", 10);
    metadata_pub = nh.advertise<norlab_basler_camera_driver::metadata_msg>("metadata", 10);
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
