#include <ros/ros.h>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <string>

#include <image_transport/image_transport.h>
#include "geometry_msgs/PoseStamped.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/Imu.h"
#include <cv_bridge/cv_bridge.h>

#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/xfeatures2d.hpp"
#include <opencv2/aruco.hpp>

#include "Euler.hpp"

Mat Prev_rot_matrix=Mat::eye(3,3,CV_64FC1);//Para calculo rotación más parecida
Mat R_axis_w2uav=Mat::eye(3,3,CV_64FC1);
EulerAngles Euler_curr;
Mat Position=Mat::zeros(3,1,CV_64FC1);

double Height;

int seq=0;
Mat img_object;
int flag_h7=0;//Bandera de fin de despegue

EulerAngles prev_angles={0,0,0};

std::ofstream file;

EulerAngles transf={0,CV_PI,CV_PI/2};
Mat Rot=ComposeRotation(transf);//Matriz de cambio de base UAV-camara

ros::Publisher Pub_estimation;


Mat CameraMatrix = (Mat_<double>(3,3) << 374.67,0,320.5,0,374.67,180.5,0,0,1);//Matriz intrínseca de la cámara

void Homography_call(Mat img_scene,ros::Time stamp);// Función de homografía
void Aruco_call(Mat img_scene,ros::Time stamp);// Función ArUCo

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
	if(flag_h7==0)
		Euler_curr=Quat2Euler(msg->orientation);//Mientras que no llegas a 7m, lees la orientación del magnetómetro


}

void ScaleCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)// Recoge el valor real de la altura y la orientación para los cambios de base
{
	Height=msg->pose.position.z;
	R_axis_w2uav=ComposeRotation(Quat2Euler(msg->pose.orientation));
	
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  if(seq==0)
  {
	seq=1;
	img_object = cv_bridge::toCvShare(msg, "mono8")->image;
  }
  else
  {
  	try
  	{
	   // Lectura de imágenes
       	  Mat img_scene = cv_bridge::toCvShare(msg, "mono8")->image;

    	  if( !img_object.data || !img_scene.data )
 	  { std::cout<< " --(!) Error reading images " << std::endl;}

	   // Distinción entre despegue y funcionamiento normal.
	  if(flag_h7==1)
	  {	Homography_call(img_scene,msg->header.stamp);	}
	  else
	  {	Aruco_call(img_scene,msg->header.stamp); 	}

	  img_scene.copyTo(img_object);
  	  cv::waitKey(30);

  	}
 	catch (cv_bridge::Exception& e)
  	{
 	   ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
 	}
  }
}

int main(int argc, char **argv)
{
  // Comandos de inicio y configuración del nodo de ROS
  ros::init(argc, argv, "image2angles");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub_img = it.subscribe("/erlecopter/bottom/image_raw", 1, imageCallback);
  ros::Subscriber sub_h = nh.subscribe("/erlecopter/imu", 1, imuCallback);
  ros::Subscriber sub_h2 = nh.subscribe("/erlecopter/ground_truth/pose", 1, ScaleCallback);
  ros::spin();
}

void Homography_call(Mat img_scene, ros::Time stamp)
{
	double Scale_factor=Height;

	  // Selección del algoritmo SURF con un umbral de 1000
	Ptr<Feature2D> fd=cv::xfeatures2d::SURF::create(1000); // Threshold=1000


	  //Detección de puntos y creación de descriptores
 	 Mat descriptors_object, descriptors_scene;
 	 std::vector<KeyPoint> keypoints_object, keypoints_scene;
 	 fd->detectAndCompute( img_object, Mat(), keypoints_object, descriptors_object );
 	 fd->detectAndCompute( img_scene, Mat(), keypoints_scene, descriptors_scene );
	

	   // Emparejamiento de descriptores con algoritmo FLANN
	  std::vector< DMatch > matches;
	  FlannBasedMatcher matcher;
	  if(keypoints_object.size()>0 && keypoints_scene.size()>0)
	  {
	  matcher.match( descriptors_object, descriptors_scene, matches );
	
	    // Nos quedamos con los mejores emparejamientos
	  double max_dist = 0; double min_dist = 100;
	    // Calculamos distancias
	  for( int i = 0; i < descriptors_object.rows; i++ )
	  { double dist = matches[i].distance;
	    if( dist < min_dist ) min_dist = dist;
	    if( dist > max_dist ) max_dist = dist;
	  }
	    //Cogemos solo aquellos con una distancia menos de un tercio de la amplitud
	  double threshold=(max_dist-min_dist)/3+min_dist;
	  std::vector< DMatch > good_matches;
	  for( int i = 0; i < descriptors_object.rows; i++ )
	  { if( matches[i].distance <= threshold )
	     { good_matches.push_back( matches[i]); }
	  }
	
	
	    // Reordena los puntos en vectores para la img anterior y la actual
	  std::vector<Point2f> obj;
	  std::vector<Point2f> scene;
	  for( size_t i = 0; i < good_matches.size(); i++ )
	  {
	    obj.push_back( keypoints_object[ good_matches[i].queryIdx ].pt );
	    scene.push_back( keypoints_scene[ good_matches[i].trainIdx ].pt );
	  }
	  
	  Mat H;
	
	    // Calcula homografía por RANSAC
	  if(!good_matches.empty())	H = findHomography( obj, scene, RANSAC );
	  
	  EulerAngles angles={0,0,0};
	  Mat tras=Mat::zeros(3,1,CV_64FC1);

	    // Extracción de desplazamientos y rotaciones
	  if (! H.empty())
	  {    
		std::vector<Mat> Rs_decomp, ts_decomp, normals_decomp;

		double Min_var=10000,Var_cost=0;
		int SOL=0;

		// Descomposición de la homografía
		int solutions = decomposeHomographyMat(H, CameraMatrix, Rs_decomp, ts_decomp, 	normals_decomp);
	
		// Quédate con la solución que aplique
		for (int i = 0; i < solutions; i++)
    		{
			if(normals_decomp[i].at<double>(2)>0)
			{
				Var_cost=FrameVar(Rs_decomp[i], Prev_rot_matrix);
				if(Var_cost<Min_var)
				{
					Min_var=Var_cost;
					SOL=i;
				}
			}			
		}

		// Calcula el desplazamiento y los ángulos girados en la base mundo.
		tras=-R_axis_w2uav*Rot*ts_decomp[SOL]*Scale_factor;
		R_axis_w2uav=(R_axis_w2uav*Rot*Rs_decomp[SOL].t()*Rot);
		angles=fast_Euler_angles(R_axis_w2uav);

		// Calcula posición
		Position.at<double>(0)+=tras.at<double>(0);
		Position.at<double>(1)+=tras.at<double>(1);
		Position.at<double>(2)+=tras.at<double>(2);

  	  }	
  	  else
  	  {	
		Prev_rot_matrix=Mat::eye(3,3,CV_64FC1);
  	  }

	  // Guarda en archivo
	  file.open("/home/javier/pruebas/MasPruebas/Data/Vars.txt", std::ios_base::app);
      	  file << stamp << ", " <<  -prev_angles.roll+angles.roll << "," << -prev_angles.pitch+angles.pitch << "," << -prev_angles.yaw+angles.yaw << "," << tras.at<double>(0) << "," << tras.at<double>(1) << "," << tras.at<double>(2) << std::endl;
	  file.close();

	  prev_angles=angles;
	  }//Final del if

}

void Aruco_call(Mat img_scene, ros::Time stamp)
{
	  // Toma rotación de la IMU
	EulerAngles Angles=Euler_curr;
	Mat rot_mat_imu=ComposeRotation(Angles);

	  // Toma como diccionario el de 6x6
 	cv::aruco::Dictionary dictionary =   cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);	

  	 std::vector<int> ids;
         std::vector<std::vector<cv::Point2f> > corners;

	   // Busca marcadores
         cv::aruco::detectMarkers(img_scene, dictionary, corners, ids);


	int i, flag23=0;
        for(i=0;i<ids.size();i++)
        {
        	if(ids[i]==23)// Si detecta el de ID 23...
		{	flag23=1;	break;	}
	}
          // ...Entonces...
	if (flag23==1)
	{
		// Estima posición del marcador con respecto a la cámara
             std::vector<cv::Vec3d> rvecs, tvecs;
             cv::aruco::estimatePoseSingleMarkers(corners, 0.45215, CameraMatrix, cv::Mat(), rvecs, tvecs);
	     
		// Cambia el resultado a cámara con respecto al marcador
	     Mat t_aux=(Mat_<double>(3,1) << tvecs[i][0], tvecs[i][1], tvecs[i][2]);
	     Position= -1*rot_mat_imu*Rot.t()*t_aux;
	
		// Si ha superado los 7m, termina el despegue
	     if(Position.at<double>(2)>7)
		flag_h7=1;
	}	
}

