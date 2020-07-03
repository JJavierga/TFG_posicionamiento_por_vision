#include "ros/ros.h"

#include "sensor_msgs/Imu.h"
#include "nav_msgs/Odometry.h"

#include <fstream>

#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"

#include "Euler.hpp"


EulerAngles Ang_prev={0, 0, 0};
ros::Publisher IMU_mod_pub;

using namespace cv;

void OrientationCallback(const nav_msgs::Odometry::ConstPtr& msg) //Toma ángulos de salida del EKF
{
	Ang_prev=Quat2Euler(msg->pose.pose.orientation);
}

void IMUCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
	
	sensor_msgs::Imu pub_msg;

	Mat Acc_wrt_prev = (Mat_<double>(3,1) << msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);

	Mat Rot = ComposeRotation(Ang_prev); // Crea matriz de rotación con los ángulos de salida del EKF

	Mat Acc_wrt_origin = Rot*Acc_wrt_prev; // Pasa aceleración a base global

	Acc_wrt_origin.at<double>(2) = Acc_wrt_origin.at<double>(2)-9.81; //Elimina gravedad

	Acc_wrt_prev = Rot.t()*Acc_wrt_origin; //Devuelve aceleración a base IMU

	// Publica los valores de la IMU calibrados y sin gravedad
	pub_msg = *msg;
	pub_msg.angular_velocity.x+=0.00054;
	pub_msg.angular_velocity.y+=-0.00585;
	pub_msg.angular_velocity.z+=0.00083;
	pub_msg.linear_acceleration.x += Acc_wrt_prev.at<double>(0)+0.18784;
	pub_msg.linear_acceleration.y += Acc_wrt_prev.at<double>(1)+0.22967;
	pub_msg.linear_acceleration.z += Acc_wrt_prev.at<double>(2)-0.11516;

	IMU_mod_pub.publish(pub_msg);	
	
}

int main(int argc, char **argv)
{
 // Configuración de nodos y topics de ROS
  ros::init(argc, argv, "imu_listener");
  ros::NodeHandle n;
  ros::Subscriber subIMU = n.subscribe("/erlecopter/imu", 1, IMUCallback);
  ros::Subscriber subEKF = n.subscribe("/odometry/filtered", 1, OrientationCallback);
  IMU_mod_pub=n.advertise<sensor_msgs::Imu>("imu_modified",2);
  ros::spin();

  return 0;
}
