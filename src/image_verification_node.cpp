#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <fstream>

#include <norlab_basler_camera_driver/metadata_msg.h>
#include <pylon/ImageDecompressor.h>
#include <pylon/PylonIncludes.h>

using namespace std;
using namespace Pylon;
using namespace cv;

// #################################################################################################################
class Verification {
    string cameraName;
    string fileName;
    public:
        Verification(string, string);
        void Callback(const norlab_basler_camera_driver::metadata_msg&);
        void VerificationFrameId(const norlab_basler_camera_driver::metadata_msg&);
        void VerificationExposureTimes(const norlab_basler_camera_driver::metadata_msg&);
    
    private:
        uint32_t totalNumberImages = 0;
        vector<uint32_t> framesId;
        uint32_t countFramesId = 0;

        vector<float32_t> usedBrackets = {1, 2, 4, 8, 16, 32};
        float confidenceInterval = 0.25;
        bool exposureTimeInBracketUsed = false;
        vector<int> exposureTimesIndexes;
        uint32_t countExposureTimes = 0;

        ofstream statusFile;
};

Verification::Verification(string camera_name, string file_name){
    cameraName = camera_name;
    fileName = file_name;
    statusFile.open(fileName+".txt");
    statusFile << "Start bagfile verification\n"; 
}

void Verification::Callback(const norlab_basler_camera_driver::metadata_msg& msg){
    VerificationFrameId(msg);
    VerificationExposureTimes(msg);
}

void Verification::VerificationFrameId(const norlab_basler_camera_driver::metadata_msg& msg){
    framesId.insert(framesId.begin(), msg.FrameId);
    if (framesId.size() == 2){
        if ((framesId[0]-framesId[1]) != 1){
            countFramesId += 1;
            cout << cameraName << ": Frame ID not consecutives. Total number lost: " << countFramesId << " (ROS Timestamp: " << msg.header.stamp << ")" << endl;
            statusFile << cameraName << ": Frame ID not consecutives. Total number lost: " << countFramesId << " (ROS Timestamp: " << msg.header.stamp << ")" << "\n";
        }
        framesId.pop_back();
    }
    totalNumberImages += 1;
}

void Verification::VerificationExposureTimes(const norlab_basler_camera_driver::metadata_msg& msg){
    for (int i = 0; i < usedBrackets.size(); i++){
        if (msg.ExposureTime >= (usedBrackets[i] - usedBrackets[i]*confidenceInterval) && msg.ExposureTime <= (usedBrackets[i] + usedBrackets[i]*confidenceInterval)){
            exposureTimesIndexes.insert(exposureTimesIndexes.begin(), i);
            exposureTimeInBracketUsed = true;
        }
    }
    if (!(exposureTimeInBracketUsed)){
        cout << cameraName << ": Image exposure time not in the brackets used. Exposure time from image: " << msg.ExposureTime << " (ROS Timestamp: " << msg.header.stamp << ")"<< endl;
        statusFile << cameraName << ": Image exposure time not in the brackets used. Exposure time from image: " << msg.ExposureTime << " (ROS Timestamp: " << msg.header.stamp << ")" << "\n";
        return;
    }
    if (exposureTimesIndexes.size() == 2){
        if ((exposureTimesIndexes[0] - exposureTimesIndexes[1]) != 1){
            if (!((exposureTimesIndexes[0] == 0 && exposureTimesIndexes[1] == (usedBrackets.size()-1)))){
                countExposureTimes += 1;
                cout << cameraName << ": Exposure Times not following good sequence. Total number lost: " << countExposureTimes << " (ROS Timestamp: " << msg.header.stamp << ")" << endl;
                cout << "ET1: " << usedBrackets[exposureTimesIndexes[0]] << "\nET2: " << usedBrackets[exposureTimesIndexes[1]] << " (ROS Timestamp: " << msg.header.stamp << ")" << endl;
                statusFile << cameraName << ": Exposure Times not following good sequence. Total number lost: " << countExposureTimes << " (ROS Timestamp: " << msg.header.stamp << ")" << "\n";
                statusFile << "ET1: " << usedBrackets[exposureTimesIndexes[0]] << "\nET2: " << usedBrackets[exposureTimesIndexes[1]] << " (ROS Timestamp: " << msg.header.stamp << ")" << "\n";
            }
        }
        exposureTimesIndexes.pop_back();
    }
    exposureTimeInBracketUsed = false;
}


// #################################################################################################################

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_verification");
    ros::NodeHandle nh;
    string saveLogFileName;
    nh.getParam("/image_verification/file_name", saveLogFileName);

    Verification camera1("Camera1", saveLogFileName);
    Verification camera2("Camera2", saveLogFileName);

    ros::Subscriber camera1_metadata_subscriber = nh.subscribe("/stereo/camera1/metadata", 1, &Verification::Callback, &camera1);
    ros::Subscriber camera2_metadata_subscriber = nh.subscribe("/stereo/camera2/metadata", 1, &Verification::Callback, &camera2);

    ros::spin();
    ROS_INFO("Image Decompression Node");
    return EXIT_SUCCESS;
}


