#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <norlab_basler_camera_driver/metadata_msg.h>
#include <pylon/ImageDecompressor.h>
#include <pylon/PylonIncludes.h>

using namespace std;
using namespace Pylon;
using namespace cv;

image_transport::Publisher out_image_camera1_pub;
image_transport::Publisher out_image_camera2_pub;


CImageDecompressor camera1_decompressor;
CImageDecompressor camera2_decompressor;

CPylonImage camera1_targetImage;
CPylonImage camera2_targetImage;

cv_bridge::CvImage out_image_msg_1;
cv_bridge::CvImage out_image_msg_2;

uint32_t totalNumberImages = 0;
vector<uint32_t> framesId;
uint32_t countFramesId = 0;

vector<float32_t> usedBrackets = {0.5, 2, 8, 16, 32};
float confidenceInterval = 0.4;
bool exposureTimeInBracketUsed = false;
vector<int> exposureTimesIndexes;
uint32_t countExposureTimes = 0;


void VerificationFrameId(const norlab_basler_camera_driver::metadata_msg& msg){
    framesId.insert(framesId.begin(), msg.imgFrameId);
    if (framesId.size() == 2){
        if ((framesId[0]-framesId[1]) != 1){
            countFramesId += 1;
            cout << "Frame ID not consecutives. Total number lost: " << countFramesId << endl;
        }
        framesId.pop_back();
    }
    totalNumberImages += 1;
}

void VerificationExposureTimes(const norlab_basler_camera_driver::metadata_msg& msg){
    for (int i = 0; i < usedBrackets.size(); i++){
        if (msg.exposureTime >= (usedBrackets[i] - usedBrackets[i]*confidenceInterval) && msg.exposureTime <= (usedBrackets[i] + usedBrackets[i]*confidenceInterval)){
            exposureTimesIndexes.insert(exposureTimesIndexes.begin(), i);
            exposureTimeInBracketUsed = true;
        }
    }
    if (!(exposureTimeInBracketUsed)){
        cout << "Image exposure time not in the brackets used. Exposure time from image: " << msg.exposureTime << endl;
    }
    if (exposureTimesIndexes.size() == 2){
        if ((exposureTimesIndexes[0] - exposureTimesIndexes[1]) != 1){
            if (!((exposureTimesIndexes[0] == 0 && exposureTimesIndexes[1] == (usedBrackets.size()-1)))){
                countExposureTimes += 1;
                cout << "Exposure Times not following good sequence. Total number lost: " << countExposureTimes << endl;
                cout << "ET1: " << usedBrackets[exposureTimesIndexes[0]] << "\nET2: " << usedBrackets[exposureTimesIndexes[1]] << endl;
            }
        }
        exposureTimesIndexes.pop_back();
    }
    exposureTimeInBracketUsed = false;
}

void metadata_callback(const norlab_basler_camera_driver::metadata_msg& msg)
{
    Mat cv_image1_RGB8_cam1;
    Mat cv_image2_RGB8_cam2;
    Mat cv_image1_bayerRG;
    Mat cv_image2_bayerRG;
    ros::Time time_now = ros::Time::now();
    camera1_decompressor = CImageDecompressor(msg.descriptor_cam1.data(), msg.descriptor_size_cam1);
    try{
        camera1_decompressor.DecompressImage(camera1_targetImage, msg.imgBuffer_cam1.data(), msg.imgSize_cam1);
    }
    catch(...){
        cout << "lost an image cam1" << endl;
    }
    out_image_msg_1.header.stamp = time_now;
    cv_image1_bayerRG = Mat(camera1_targetImage.GetHeight(), camera1_targetImage.GetWidth(), CV_16UC1, (uint16_t *) camera1_targetImage.GetBuffer());
    //cv_image1_bayerRG = Mat(camera1_targetImage.GetHeight(), camera1_targetImage.GetWidth(), CV_8UC1, (uint8_t *) camera1_targetImage.GetBuffer());

    Mat cv_image1_RGB16(cv_image1_bayerRG.cols, cv_image1_bayerRG.rows, CV_16UC3);
    cvtColor(cv_image1_bayerRG, cv_image1_RGB16, COLOR_BayerRG2RGB);
    cv_image1_RGB16.convertTo(cv_image1_RGB8_cam1, CV_8UC3, 1.0/16);
    out_image_msg_1.image = cv_image1_RGB8_cam1;

    //Mat cv_image1_RGB8(cv_image1_bayerRG.cols, cv_image1_bayerRG.rows, CV_8UC3);
    //cvtColor(cv_image1_bayerRG, cv_image1_RGB8, COLOR_BayerRG2RGB);
    //out_image_msg_1.image = cv_image1_RGB8;
 
    // cout << "############################" << endl;
    // cout << "Camera 1 timestamp: " << msg.cameraTimestamp << endl;
    // cout << "Camera 1 frame id: " << msg.imgFrameId << endl;
    // cout << "Camera 1 exposure time: " << msg.exposureTime << endl;
    // cout << "Camera 1 mean value: " << mean(mean(cv_image1_RGB8_cam1)) << endl;

    VerificationFrameId(msg);
    VerificationExposureTimes(msg);


    camera2_decompressor = CImageDecompressor(msg.descriptor_cam2.data(), msg.descriptor_size_cam2);
    try{
        camera2_decompressor.DecompressImage(camera2_targetImage, msg.imgBuffer_cam2.data(), msg.imgSize_cam2);
    }
    catch(...){
        cout << "lost an image cam2" << endl;
    }
    out_image_msg_2.header.stamp = time_now;
    cv_image2_bayerRG = Mat(camera2_targetImage.GetHeight(), camera2_targetImage.GetWidth(), CV_16UC1, (uint16_t *) camera2_targetImage.GetBuffer());
    //cv_image2_bayerRG = Mat(camera2_targetImage.GetHeight(), camera2_targetImage.GetWidth(), CV_8UC1, (uint8_t *) camera2_targetImage.GetBuffer());

    Mat cv_image2_RGB16(cv_image2_bayerRG.cols, cv_image2_bayerRG.rows, CV_16UC3);
    cvtColor(cv_image2_bayerRG, cv_image2_RGB16, COLOR_BayerRG2RGB);
    cv_image2_RGB16.convertTo(cv_image2_RGB8_cam2, CV_8UC3, 1.0/16);
    out_image_msg_2.image = cv_image2_RGB8_cam2;

    //Mat cv_image2_RGB8(cv_image2_bayerRG.cols, cv_image2_bayerRG.rows, CV_8UC3);
    //cvtColor(cv_image2_bayerRG, cv_image2_RGB8, COLOR_BayerRG2RGB);
    //out_image_msg_2.image = cv_image2_RGB8;


    out_image_camera1_pub.publish(out_image_msg_1.toImageMsg());
    out_image_camera2_pub.publish(out_image_msg_2.toImageMsg());
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_decompression");
    ros::NodeHandle nh;
    image_transport::ImageTransport it_cam1(nh);
    image_transport::ImageTransport it_cam2(nh);
    out_image_camera1_pub = it_cam1.advertise("/camera1/image_decompression", 10);
    out_image_camera2_pub = it_cam2.advertise("/camera2/image_decompression", 10);

    out_image_msg_1.header.frame_id = "Camera1";
    out_image_msg_2.header.frame_id = "Camera2";
    out_image_msg_1.encoding = sensor_msgs::image_encodings::BGR8;
    out_image_msg_2.encoding = sensor_msgs::image_encodings::BGR8;
    

    ros::Subscriber metadata_subscriber = nh.subscribe("/stereo/image_metapackets", 1, metadata_callback);

    ros::spin();
    ROS_INFO("Image Decompression Node");
    return EXIT_SUCCESS;
}

