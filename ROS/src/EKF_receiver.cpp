#include <ros/ros.h>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include "nav_msgs/Odometry.h"
#include "Euler.hpp"

std::ofstream file;

void Callback(const nav_msgs::Odometry::ConstPtr& msg)
{

	EulerAngles a = Quat2Euler(msg->pose.pose.orientation);
	geometry_msgs::Point p = msg->pose.pose.position;

	file.open("/home/javier/pruebas/MasPruebas/Data/EKF_data.txt", std::ios_base::app);
	file << msg->header.stamp << "," << a.roll << "," << a.pitch << "," << a.yaw << "," << p.x << "," << p.y << "," << p.z << std::endl;
	file.close();

}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "EKF_receiver");
  ros::NodeHandle nh;
  ros::Subscriber sub_h = nh.subscribe("/odometry/filtered", 10, Callback);
  ros::spin();
}
