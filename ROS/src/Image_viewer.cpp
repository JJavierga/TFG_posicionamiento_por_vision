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


void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
	// Muestra imágenes recibidas
    cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);

    cv::waitKey(30);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

int main(int argc, char **argv)
{
  //Configuracion nodos y topics de ROS y ventana de visión de imágenes
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;
  cv::namedWindow("view");
  cv::startWindowThread();
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("/erlecopter/bottom/image_raw", 1, imageCallback);
  ros::spin();
  cv::destroyWindow("view");
}
