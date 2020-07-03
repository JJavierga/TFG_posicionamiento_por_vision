#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgcodecs.hpp>
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include <string.h>
#include <sstream>
#include <fstream>

int seq;
std::ofstream file;

template <typename T>
std::string ToString(T val)
{
    std::stringstream stream;
    stream << val;
    return stream.str();
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
    cv::imwrite("/home/javier/pruebas/MasPruebas/Imagenes/" + ToString(seq) + ".jpg", cv_bridge::toCvShare(msg, "bgr8")->image);
    seq++;

    file.open("/home/javier/pruebas/MasPruebas/Data/Image_timing.txt", std::ios_base::app);
    file << msg->header.stamp << std::endl;
    file.close();

    cv::waitKey(30);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;
  cv::namedWindow("view");
  cv::startWindowThread();
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("/erlecopter/bottom/image_raw", 1, imageCallback);
  ros::spin();
  cv::destroyWindow("view");
}
