#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include <fstream>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "Euler.hpp"




std::ofstream file;
Mat R_acum=Mat::eye(3,3,CV_64FC1);
geometry_msgs::Vector3 Vel_prev_wrt_origin;
geometry_msgs::Vector3 Pos_wrt_origin;
uint32_t seq=0;

using namespace cv;


void chatterCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
if(seq==0)
	{
	seq++;
	Vel_prev_wrt_origin.x=0;Vel_prev_wrt_origin.y=0;Vel_prev_wrt_origin.z=0;
	Pos_wrt_origin.x=0;Pos_wrt_origin.y=0;Pos_wrt_origin.z=0;
	}
else
	{
	EulerAngles euler_angles, Rot_curr_wrt_prev;
	double interval=0.0025;
	
	// Orientation estimation:
	Rot_curr_wrt_prev.roll=msg->angular_velocity.x*interval;
	Rot_curr_wrt_prev.pitch=msg->angular_velocity.y*interval;
	Rot_curr_wrt_prev.yaw=msg->angular_velocity.z*interval;

	R_acum=R_acum*ComposeRotation(Rot_curr_wrt_prev);
	
	euler_angles=fast_Euler_angles(R_acum);

	// Position estimation:
	Mat Acc_curr_wrt_curr = (Mat_<double>(3,1) << msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
	Mat Acc_curr_wrt_origin=R_acum*Acc_curr_wrt_curr;
	
	Pos_wrt_origin.x+=(Acc_curr_wrt_origin.at<double>(0))*interval*interval*0.5+Vel_prev_wrt_origin.x*interval;
	Pos_wrt_origin.y+=(Acc_curr_wrt_origin.at<double>(1))*interval*interval*0.5+Vel_prev_wrt_origin.y*interval;
	Pos_wrt_origin.z+=(Acc_curr_wrt_origin.at<double>(2))*interval*interval*0.5+Vel_prev_wrt_origin.z*interval;

	// Velocities update
	Vel_prev_wrt_origin.x=Acc_curr_wrt_origin.at<double>(0)*interval+Vel_prev_wrt_origin.x;
	Vel_prev_wrt_origin.y=Acc_curr_wrt_origin.at<double>(1)*interval+Vel_prev_wrt_origin.y;
	Vel_prev_wrt_origin.z=Acc_curr_wrt_origin.at<double>(2)*interval+Vel_prev_wrt_origin.z;

	seq++;

	file.open("/home/javier/pruebas/MasPruebas/Data/data_imu.txt", std::ios_base::app);
   	file << msg->header.stamp << "," << euler_angles.roll << "," << euler_angles.pitch << "," << euler_angles.yaw  << "," << Pos_wrt_origin.x << "," << Pos_wrt_origin.y << "," << Pos_wrt_origin.z << std::endl;
	file.close();

	}
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "imu_listener");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/erlecopter/imu", 1000, chatterCallback);
  ros::spin();

  return 0;
}
