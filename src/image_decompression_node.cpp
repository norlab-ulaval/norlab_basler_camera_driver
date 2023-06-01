#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <norlab_basler_camera_driver/packets_msg.h>
#include <pylon/ImageDecompressor.h>
#include <pylon/PylonIncludes.h>

using namespace std;
using namespace Pylon;
using namespace cv;

// #################################################################################################################
class Camera {
    string cameraName;
    int bits;
    image_transport::Publisher out_pub;
    CImageDecompressor camera_decompressor;
    CPylonImage targetImage;
    cv_bridge::CvImage out_image_msg;

    public:
        Camera(string, int, image_transport::Publisher, CImageDecompressor&, CPylonImage&, cv_bridge::CvImage);
        void ImageDecompressorCallback(const norlab_basler_camera_driver::packets_msg&);
        void FormatImagesDecompressed(const norlab_basler_camera_driver::packets_msg&, Mat&, Mat&);
};

Camera::Camera(string name, int numBits, image_transport::Publisher topic_out, CImageDecompressor& decompressor, CPylonImage& image, cv_bridge::CvImage out_image){
    cameraName = name;
    bits = numBits;
    out_pub = topic_out;
    camera_decompressor = decompressor;
    targetImage = image;
    out_image_msg = out_image;
}

void Camera::ImageDecompressorCallback(const norlab_basler_camera_driver::packets_msg& msg)
{
    Mat cvOuputImage;
    Mat cvInputImage;
    FormatImagesDecompressed(msg, cvInputImage, cvOuputImage);
    sensor_msgs::ImagePtr img_msg = out_image_msg.toImageMsg();
    img_msg->header.stamp = msg.header.stamp;
    out_pub.publish(*img_msg);
}

void Camera::FormatImagesDecompressed(const norlab_basler_camera_driver::packets_msg& msg, Mat& cvInputImage, Mat& cvOuputImage){
    camera_decompressor = CImageDecompressor(msg.descriptor.data(), msg.descriptor_size);
    try{
        camera_decompressor.DecompressImage(targetImage, msg.imgBuffer.data(), msg.imgSize);
    }
    catch(...){
        cout << cameraName << ": lost an image due to compression issue" << endl;
    }
    // out_image_msg.header.stamp = msg.header.stamp;
    cvInputImage = Mat(targetImage.GetHeight(), targetImage.GetWidth(), CV_16UC1, (uint16_t *) targetImage.GetBuffer());
    if (bits == 12){
        cvOuputImage = cvInputImage;
    }
    else if (bits == 8)
    {
        Mat cv_image_RGB16(cvInputImage.cols, cvInputImage.rows, CV_16UC3);
        cvtColor(cvInputImage, cv_image_RGB16, COLOR_BayerRG2RGB);
        cv_image_RGB16.convertTo(cvOuputImage, CV_8UC3, 1.0/16);
    }
    out_image_msg.image = cvOuputImage;
}

// #################################################################################################################

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_decompression");
    ros::NodeHandle nh;
    image_transport::ImageTransport it_cam1(nh);
    image_transport::ImageTransport it_cam2(nh);
    int numberBits;
    nh.getParam("/image_decompression/number_of_bits_output", numberBits);

    image_transport::Publisher out_image_camera1_pub;
    image_transport::Publisher out_image_camera2_pub;
    CImageDecompressor camera1_decompressor;
    CImageDecompressor camera2_decompressor;
    CPylonImage camera1_targetImage;
    CPylonImage camera2_targetImage;
    cv_bridge::CvImage out_image_msg_1;
    cv_bridge::CvImage out_image_msg_2;

    out_image_camera1_pub = it_cam1.advertise("/camera1/image_decompressed", 10);
    out_image_camera2_pub = it_cam2.advertise("/camera2/image_decompressed", 10);

    out_image_msg_1.header.frame_id = "Camera1";
    out_image_msg_2.header.frame_id = "Camera2";
    if (numberBits == 12){
        out_image_msg_1.encoding = sensor_msgs::image_encodings::BAYER_RGGB16;
        out_image_msg_2.encoding = sensor_msgs::image_encodings::BAYER_RGGB16;
    }
    else if (numberBits == 8)
    {
        out_image_msg_1.encoding = sensor_msgs::image_encodings::BGR8;
        out_image_msg_2.encoding = sensor_msgs::image_encodings::BGR8;
    }

    Camera camera1("Camera1", numberBits, out_image_camera1_pub, camera1_decompressor, camera1_targetImage, out_image_msg_1);
    Camera camera2("Camera2", numberBits, out_image_camera2_pub, camera2_decompressor, camera2_targetImage, out_image_msg_2);

    ros::Subscriber camera1_packets_subscriber = nh.subscribe("/stereo/camera1/image_compressed", 1, &Camera::ImageDecompressorCallback, &camera1);
    ros::Subscriber camera2_packets_subscriber = nh.subscribe("/stereo/camera2/image_compressed", 1, &Camera::ImageDecompressorCallback, &camera2);

    ros::spin();
    ROS_INFO("Image Decompression Node");
    return EXIT_SUCCESS;
}

