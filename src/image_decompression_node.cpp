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

Mat cv_image1_bayerRG;
Mat cv_image2_bayerRG;
CImageDecompressor camera1_decompressor;
CImageDecompressor camera2_decompressor;

CPylonImage camera1_targetImage;
CPylonImage camera2_targetImage;

cv_bridge::CvImage out_image_msg_1;
cv_bridge::CvImage out_image_msg_2;

void metadata_callback(const norlab_basler_camera_driver::metadata_msg& msg)
{
    ros::Time time_now = ros::Time::now();
    camera1_decompressor = CImageDecompressor(msg.descriptor_cam1.data(), msg.descriptor_size_cam1);
    camera1_decompressor.DecompressImage(camera1_targetImage, msg.imgBuffer_cam1.data(), msg.imgSize_cam1);
    out_image_msg_1.header.stamp = time_now;
    out_image_msg_1.image = Mat(camera1_targetImage.GetHeight(), camera1_targetImage.GetWidth(), CV_8UC1, (uint8_t *) camera1_targetImage.GetBuffer());

    camera2_decompressor = CImageDecompressor(msg.descriptor_cam2.data(), msg.descriptor_size_cam2);
    camera2_decompressor.DecompressImage(camera2_targetImage, msg.imgBuffer_cam2.data(), msg.imgSize_cam2);
    out_image_msg_2.header.stamp = time_now;
    out_image_msg_2.image = Mat(camera2_targetImage.GetHeight(), camera2_targetImage.GetWidth(), CV_8UC1, (uint8_t *) camera2_targetImage.GetBuffer());

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
    out_image_msg_1.encoding = sensor_msgs::image_encodings::BAYER_RGGB8;
    out_image_msg_2.encoding = sensor_msgs::image_encodings::BAYER_RGGB8;
    

    ros::Subscriber metadata_subscriber = nh.subscribe("/stereo/image_metapackets", 1, metadata_callback);

    ros::spin();
    ROS_INFO("Image Decompression Node");
    return EXIT_SUCCESS;
}