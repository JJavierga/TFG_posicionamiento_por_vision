#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Vector3.h"
#include <fstream>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "Euler.hpp"


Mat R_acum=Mat::eye(3,3,CV_64FC1);
geometry_msgs::Point Vel_prev_wrt_origin;
geometry_msgs::Point Pos_wrt_origin;
uint32_t seq=0;

ros::Publisher Pub_estimation;

using namespace cv;


void chatterCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
if(seq==0)
	{
	// prev_time=msg->header.stamp;
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

	geometry_msgs::PoseStamped new_msg;
	
	new_msg.header.seq=seq;
	new_msg.header.stamp=msg->header.stamp;
	
	new_msg.pose.orientation=Euler2Quat(euler_angles);
	new_msg.pose.position=Pos_wrt_origin;

	Pub_estimation.publish(new_msg);

	}
}

int main(int argc, char **argv)
{
   // Configuración nodos y topics de ROS
  ros::init(argc, argv, "imu_listener");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("/imu_modified", 1000, chatterCallback);

  Pub_estimation=n.advertise<geometry_msgs::PoseStamped>("/imu_estimation",1);

  ros::spin();

  return 0;
}
