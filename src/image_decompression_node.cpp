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
    int bitsInput;
    int bitsOutput;
    image_transport::Publisher out_pub;
    CImageDecompressor camera_decompressor;
    CPylonImage targetImage;
    cv_bridge::CvImage out_image_msg;

    public:
        Camera(string, int, int, image_transport::Publisher, CImageDecompressor&, CPylonImage&, cv_bridge::CvImage);
        void ImageDecompressorCallback(const norlab_basler_camera_driver::packets_msg&);
        void FormatImagesDecompressed(const norlab_basler_camera_driver::packets_msg&, Mat&, Mat&);
};

Camera::Camera(string name, int numBitsInput, int numBitsOutput, image_transport::Publisher topic_out, CImageDecompressor& decompressor, CPylonImage& image, cv_bridge::CvImage out_image){
    cameraName = name;
    bitsInput = numBitsInput;
    bitsOutput = numBitsOutput;
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
    if (bitsInput == 12){
        cvInputImage = Mat(targetImage.GetHeight(), targetImage.GetWidth(), CV_16UC1, (uint16_t *) targetImage.GetBuffer());
    }
    else if (bitsInput == 8)
    {
        cvInputImage = Mat(targetImage.GetHeight(), targetImage.GetWidth(), CV_8UC1, (uint8_t *) targetImage.GetBuffer());
    }
    
    if (bitsInput == bitsOutput){
        if (bitsInput == 12){
            cvOuputImage = cvInputImage;
        }
        else if (bitsInput == 8)
        {
            // Mat cv_image_RGB8(cvInputImage.cols, cvInputImage.rows, CV_8UC3);
            cvtColor(cvInputImage, cvOuputImage, COLOR_BayerRG2RGB);
            // cv_image_RGB16.convertTo(cvOuputImage, CV_8UC3, 1.0/16);
        }
    }
    else if (bitsInput == 12 && bitsOutput == 8)
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
    int numberBitsInput;
    int numberBitsOutput;
    nh.getParam("/image_decompression/number_of_bits_input", numberBitsInput);
    nh.getParam("/image_decompression/number_of_bits_output", numberBitsOutput);

    image_transport::Publisher out_image_camera1_pub;
    image_transport::Publisher out_image_camera2_pub;
    CImageDecompressor camera1_decompressor;
    CImageDecompressor camera2_decompressor;
    CPylonImage camera1_targetImage;
    CPylonImage camera2_targetImage;
    cv_bridge::CvImage out_image_msg_1;
    cv_bridge::CvImage out_image_msg_2;

    out_image_camera1_pub = it_cam1.advertise("/stereo/camera1/image_decompressed", 10);
    out_image_camera2_pub = it_cam2.advertise("/stereo/camera2/image_decompressed", 10);

    out_image_msg_1.header.frame_id = "camera1_link";
    out_image_msg_2.header.frame_id = "camera2_link";
    if (numberBitsOutput == 12){
        out_image_msg_1.encoding = sensor_msgs::image_encodings::BAYER_RGGB16;
        out_image_msg_2.encoding = sensor_msgs::image_encodings::BAYER_RGGB16;
    }
    else if (numberBitsOutput == 8)
    {
        out_image_msg_1.encoding = sensor_msgs::image_encodings::BGR8;
        out_image_msg_2.encoding = sensor_msgs::image_encodings::BGR8;
    }

    Camera camera1("Camera1", numberBitsInput, numberBitsOutput, out_image_camera1_pub, camera1_decompressor, camera1_targetImage, out_image_msg_1);
    Camera camera2("Camera2", numberBitsInput, numberBitsOutput, out_image_camera2_pub, camera2_decompressor, camera2_targetImage, out_image_msg_2);

    ros::Subscriber camera1_packets_subscriber = nh.subscribe("/stereo/camera1/image_compressed", 1, &Camera::ImageDecompressorCallback, &camera1);
    ros::Subscriber camera2_packets_subscriber = nh.subscribe("/stereo/camera2/image_compressed", 1, &Camera::ImageDecompressorCallback, &camera2);

    ros::spin();
    ROS_INFO("Image Decompression Node");
    return EXIT_SUCCESS;
}

