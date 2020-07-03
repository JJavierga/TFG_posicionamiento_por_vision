#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include <fstream>
#include <cmath>
#include "Euler.hpp"

std::ofstream file;


void chatterCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{

	EulerAngles euler_angles;
	geometry_msgs::Quaternion message;

	message.x=msg->pose.orientation.x;	
	message.y=msg->pose.orientation.y;
	message.z=msg->pose.orientation.z;
	message.w=msg->pose.orientation.w;

	euler_angles=Quat2Euler(message);


	file.open("/home/javier/pruebas/MasPruebas/Data/ground_truth.txt", std::ios_base::app);
   	file << msg->header.stamp << "," << euler_angles.roll << "," << euler_angles.pitch << "," << euler_angles.yaw << "," << msg->pose.position.x << "," << msg->pose.position.y << "," << msg->pose.position.z << std::endl;
	file.close();

}

int main(int argc, char **argv)
{
   // Configuracion nodos y topics de ROS
  ros::init(argc, argv, "ground_truth_listener");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("/erlecopter/ground_truth/pose", 1000, chatterCallback);

  ros::spin();

  return 0;
}
