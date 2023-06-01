#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <norlab_basler_camera_driver/metadata_msg.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

using namespace std;
using namespace cv;

// #################################################################################################################
class Camera {
    string cameraName;
    ros::NodeHandle nh;
    std::vector<int32_t> bracketsUsed;
    std::vector<image_transport::Publisher> outputPublisherVector;
    std::vector<sensor_msgs::Image> outputMsgVector;
    sensor_msgs::Image outMsg;

    std::unique_ptr<message_filters::Subscriber<sensor_msgs::Image>> imageSub;
    std::unique_ptr<message_filters::Subscriber<norlab_basler_camera_driver::metadata_msg>> metadataSub;
    std::unique_ptr<message_filters::TimeSynchronizer<sensor_msgs::Image, norlab_basler_camera_driver::metadata_msg>> syncSubs;

    public:
        Camera(string, ros::NodeHandle);
        void ImageSeparatorCallback(const sensor_msgs::Image::ConstPtr&, const norlab_basler_camera_driver::metadata_msg::ConstPtr&);
        int FindIndexBracket(const norlab_basler_camera_driver::metadata_msg&);
};

Camera::Camera(string name, ros::NodeHandle nhInput){
    cameraName = name;
    nh = nhInput;
    image_transport::ImageTransport it_cam1(nh);

    for(int i = 0; i < 6; i++){
        outputPublisherVector.push_back(it_cam1.advertise("/camera1/image_bracket" + to_string(i+1), 10));
    }

    bracketsUsed = {1, 2, 4, 8, 16, 32};

    imageSub = std::make_unique<message_filters::Subscriber<sensor_msgs::Image>>(nh, "/camera1/image_decompressed", 10);
    metadataSub = std::make_unique<message_filters::Subscriber<norlab_basler_camera_driver::metadata_msg>>(nh, "/stereo/camera1/metadata", 10);
    syncSubs = std::make_unique<message_filters::TimeSynchronizer<sensor_msgs::Image, norlab_basler_camera_driver::metadata_msg>>(*imageSub, *metadataSub, 10);
    cout << "test avant" << endl;
    syncSubs->registerCallback(boost::bind(&Camera::ImageSeparatorCallback, this, _1, _2));
}

void Camera::ImageSeparatorCallback(const sensor_msgs::Image::ConstPtr& image_msg, const norlab_basler_camera_driver::metadata_msg::ConstPtr& metadata_msg)
{
    int bracketIndex = FindIndexBracket(*metadata_msg);
    outMsg.header = image_msg->header;
    outMsg.height = image_msg->height;
    outMsg.width = image_msg->width;
    outMsg.encoding = image_msg->encoding;
    outMsg.data = image_msg->data;
    outputPublisherVector[bracketIndex].publish(outMsg);
}

int Camera::FindIndexBracket(const norlab_basler_camera_driver::metadata_msg& msg){
    int index = 100;
    float confidenceInterval = 0.25;
    for (int i = 0; i < bracketsUsed.size(); i++){
        if (msg.ExposureTime >= (bracketsUsed[i] - bracketsUsed[i]*confidenceInterval) && msg.ExposureTime <= (bracketsUsed[i] + bracketsUsed[i]*confidenceInterval)){
            index = i;
        }
    }
    return index;
}
// #################################################################################################################

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_separator");
    ros::NodeHandle nh;

    Camera camera1("Camera1", nh);

    // message_filters::Subscriber<sensor_msgs::Image> imageSub(nh, "/stereo/camera1/image_decompressed", 1);
    // message_filters::Subscriber<norlab_basler_camera_driver::metadata_msg> metadataSub(nh, "", 1);
    // message_filters::TimeSynchronizer<sensor_msgs::Image, norlab_basler_camera_driver::metadata_msg> syncSubs(imageSub, metadataSub, 10);
    // syncSubs.registerCallback(boost::bind(&Camera::ImageSeparatorCallback, _1, _2), &camera1);


    // ros::Subscriber camera1_packets_subscriber = nh.subscribe("/stereo/camera1/image_compressed", 1, &Camera::ImageDecompressorCallback, &camera1);

    ros::spin();
    ROS_INFO("Image Separator Node");
    return EXIT_SUCCESS;
}

