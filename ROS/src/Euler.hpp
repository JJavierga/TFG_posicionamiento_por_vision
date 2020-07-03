#include <opencv2/imgcodecs.hpp>
#include "opencv2/core.hpp"
#include <stdio.h>
#include <iostream>
#include <opencv2/core/mat.hpp>
#include <cstdlib>
#include <geometry_msgs/Quaternion.h>
#include "opencv2/calib3d.hpp"

using namespace cv;


struct EulerAngles {
    double roll, pitch, yaw;
};


EulerAngles Quat2Euler(geometry_msgs::Quaternion q) {
    EulerAngles angles;
    double discriminante = q.w * q.y - q.x * q.z;

    if(discriminante<0.4999 && discriminante>-0.4999)
    {
    	// roll (x-axis rotation)
    	double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
    	double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    	angles.roll = std::atan2(sinr_cosp, cosr_cosp);

    	// pitch (y-axis rotation)
    	double sinp = 2 * (q.w * q.y - q.z * q.x);
    	/*if (std::abs(sinp) >= 1)
        	angles.pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    	else*/
        angles.pitch = std::asin(sinp);

    	// yaw (z-axis rotation)
    	double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    	double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    	angles.yaw = std::atan2(siny_cosp, cosy_cosp);
    }
    else if(discriminante>0.4999)
    {
	angles.roll=0;
	angles.pitch=CV_PI/2;
	angles.yaw=-2*atan2(q.x,q.w);
    }
    else if(discriminante<-0.4999)
    {
	angles.roll=0;
	angles.pitch=-CV_PI/2;
	angles.yaw=2*atan2(q.x,q.w);
    }
    return angles;
}

Mat ComposeRotation(EulerAngles a)
{
	Mat Rx = (Mat_<double>(3,3) << 1, 0, 0, 0, cos(a.roll), -sin(a.roll), 0, sin(a.roll), cos(a.roll));
	Mat Ry = (Mat_<double>(3,3) << cos(a.pitch), 0, sin(a.pitch), 0, 1, 0, -sin(a.pitch), 0, cos(a.pitch));
	Mat Rz = (Mat_<double>(3,3) << cos(a.yaw), -sin(a.yaw), 0, sin(a.yaw), cos(a.yaw), 0, 0, 0, 1);

	Mat R=Rz*Ry*Rx;
	
	return R;
}

geometry_msgs::Quaternion Euler2Quat(EulerAngles a)
{
	geometry_msgs::Quaternion q;

	a.roll=a.roll/2;
	a.pitch=a.pitch/2;
	a.yaw=a.yaw/2;

	q.w=cos(a.roll)*cos(a.pitch)*cos(a.yaw)+sin(a.roll)*sin(a.pitch)*sin(a.yaw);
	q.x=sin(a.roll)*cos(a.pitch)*cos(a.yaw)-cos(a.roll)*sin(a.pitch)*sin(a.yaw);
	q.y=cos(a.roll)*sin(a.pitch)*cos(a.yaw)+sin(a.roll)*cos(a.pitch)*sin(a.yaw);
	q.z=cos(a.roll)*cos(a.pitch)*sin(a.yaw)-sin(a.roll)*sin(a.pitch)*cos(a.yaw);

	return q;
}

EulerAngles fast_Euler_angles(Mat R) {

    EulerAngles output; // [x,y,z]

    // comprobar si cos(theta)=0
    if (R.at<double>(0,2) < -0.99999)
    {
        output.pitch = 0; 
        output.roll = CV_PI / 2;
        output.yaw = output.roll + atan2(R.at<double>(1,0), R.at<double>(2,0));
    }
    else if (R.at<double>(0,2) > 0.99999) 
    {
        output.pitch = 0;
        output.roll = -CV_PI / 2;
        output.yaw = -output.roll + atan2(-R.at<double>(1,0), -R.at<double>(2,0));
    }
    else
    { // Existen 2 soluciones, pero solo queremos aquella no que sea mayor de pi/2
	double c;

        output.pitch = -asin(R.at<double>(2,0));
	
	c=cos(output.pitch);	

      	output.roll = atan2(R.at<double>(2,1) / c, R.at<double>(2,2) / c);

      	output.yaw = atan2(R.at<double>(1,0) / c, R.at<double>(0,0) / c);

	//std::cout << "Posibles soluciones:" << std::endl << x1 << " " << y1 << " " << z1 << std::endl << x2 << " " << y2 << " " << z2 << std::endl; 	
    }

    return output;

}

double FrameVar(Mat Rot_Curr, Mat Rot_Prev)
{
	//Previous rotation: Rot_Prev ; current rotation: Rot_Curr
	double NewAngle, OldAngle, aux;
	Mat Rot_Curr_vect, Rot_Prev_vect;
	Mat VarAngle(cv::Size(1,1), CV_64FC1);

	Rodrigues(Rot_Curr, Rot_Curr_vect);
	Rodrigues(Rot_Prev, Rot_Prev_vect);
	Rot_Prev_vect=Rot_Prev.inv()*Rot_Prev_vect;

	VarAngle=(Rot_Curr_vect-Rot_Prev_vect).dot((Rot_Curr_vect-Rot_Prev_vect));
	
	return VarAngle.at<double>(0);
}
