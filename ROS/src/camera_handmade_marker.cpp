#include <ros/ros.h>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <string>
#include <image_transport/image_transport.h>
#include "geometry_msgs/PoseStamped.h"
#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgcodecs.hpp>
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "Euler.hpp"
#include <opencv2/aruco.hpp>

std::ofstream file;
EulerAngles angles;

EulerAngles transf={0,CV_PI,CV_PI/2};
const Mat Rot=ComposeRotation(transf); //Matriz de cambio de base UAV-camara

const Mat CameraMatrix = (Mat_<double>(3,3) << 374.67,0,320.5,0,374.67,180.5,0,0,1);//Matriz intrínseca de la cámara

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{

  	try
  	{
	  // Lectura de imágenes
  	 Mat img_scene = cv_bridge::toCvShare(msg, "mono8")->image;
 	 if(!img_scene.data )
 	 { std::cout<< " --(!) Error reading images " << std::endl;}

	  // Toma como diccionario el de 6x6
 	cv::aruco::Dictionary dictionary =   cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);	

  	 std::vector<int> ids;
         std::vector<std::vector<cv::Point2f> > corners;

	  // Busca marcadores
         cv::aruco::detectMarkers(img_scene, dictionary, corners, ids);

	
   	 // Busca entre eso al marcador de ID 23
	int i, flag23=0;
        for(i=0;i<ids.size();i++)
        {
        	if(ids[i]==23)
		{	flag23=1;	break;	}
	}
        
	  // SI se detecta...
	if (flag23==1)
	  {
		// Estima posición del marcador con respecto a la cámara
             std::vector<cv::Vec3d> rvecs, tvecs;
             cv::aruco::estimatePoseSingleMarkers(corners, 0.45215, CameraMatrix, cv::Mat(), rvecs, tvecs);
	     
		// Extrae la rotación del marcador
	     Mat rot_mat;
	     Rodrigues(rvecs[i],rot_mat);
	     angles=fast_Euler_angles(rot_mat.t()*Rot);
	
		// Crea un Mat con la traslación
	     Mat t_aux=(Mat_<double>(3,1) << tvecs[i][0], tvecs[i][1], tvecs[i][2]);
	     Mat tras= t_aux;
	     
		// Almacena la traslación con el cambio a mano de las coords X e Y.
	     file.open("/home/javier/pruebas/MasPruebas/Data/Marker_pose.txt", std::ios_base::app);
      	     file << msg->header.stamp << ", " <<  angles.roll << "," << angles.pitch << "," << angles.yaw << "," << tras.at<double>(1) << "," <<  tras.at<double>(0) << "," << tras.at<double>(2) << std::endl;
	     file.close();

         }


	 cv::waitKey(30);

  	}
 	catch (cv_bridge::Exception& e)
  	{
 	   ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
 	}
  
}

int main(int argc, char **argv)
{
  // Comandos de inicio y configuración del nodo de ROS
  ros::init(argc, argv, "Marker_image2pose");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub_img = it.subscribe("/erlecopter/bottom/image_raw", 1, imageCallback);
  ros::spin();
}
