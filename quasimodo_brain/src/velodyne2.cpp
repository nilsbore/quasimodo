
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>

// PCL specific includes
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "modelupdater/ModelUpdater.h"
#include "core/RGBDFrame.h"

#include <string.h>

using namespace std;

int counterr = 0;
int counterl = 0;
std::string path = "./";

void  cloud_cb_l(const sensor_msgs::PointCloud2ConstPtr& input){
	ROS_INFO("l pointcloud in %i",counterl);
}

pcl::PointCloud<pcl::PointXYZRGB> fullcloudr;
void  cloud_cb_r(const sensor_msgs::PointCloud2ConstPtr& input){
	ROS_INFO("r pointcloud in %i",counterr);

	pcl::PointCloud<pcl::PointXYZRGB> cloud;
	pcl::fromROSMsg (*input, cloud);

//	for(int i = 0; i < cloud.points.size(); i++){
//		fullcloudr.points.push_back(cloud.points[i]);
//	}
	int r [16];
	int g [16];
	int b [16];
	for(int h = 0; h < 16; h++){
		r[h] = rand() % 256;
		g[h] = rand() % 256;
		b[h] = rand() % 256;
	}

	for(int i = 0; i < cloud.points.size(); i++){
		cloud.points[i].r = r[i%16];
		cloud.points[i].g = g[i%16];
		cloud.points[i].b = b[i%16];
	}


//	cloud.width = cloud.points.size()/16;
//	int height = 16;
//	int width = cloud.points.size()/16;

//	for(int h = 0; h < 16; h++){
//		int r = rand() % 256;
//		int g = rand() % 256;
//		int b = rand() % 256;
//		for(int w = 0; w < cloud.width; w++){
//			int i = h*cloud.width+w;
//			cloud.points[i].r = r;
//			cloud.points[i].g = g;
//			cloud.points[i].b = b;
//		}
//	}

//	for(int i = 0; i < cloud.points.size(); i++){
//		cloud.points[i].r = (i%16);
//		cloud.points[i].g = (i%16);
//		cloud.points[i].b = 255;
//	}
//	if(counterr % 2 == 1){
//		char buf[1024];
//		fullcloudr.height = 16; fullcloudr.width = fullcloudr.points.size()/fullcloudr.height;
//		sprintf(buf,"%s/right_%.10i.pcd",path.c_str(),counterr/2);
//		//pcl::io::savePCDFileBinary (string(buf), fullcloudr);
//		printf("nr points : %i resolution %i %i\n",fullcloudr.points.size(),cloud.width,cloud.height);
//		fullcloudr.points.resize(0);
//	}

	char buf[1024];
	sprintf(buf,"%s/right_%.10i.pcd",path.c_str(),counterr);
	pcl::io::savePCDFileBinary (string(buf), cloud);

//	char buf[1024];
//	sprintf(buf,"%s/right_%.10i.pcd",path.c_str(),counterr);
	//printf("nr points : %i resolution %i %i\n",fullcloudr.points.size(),cloud.width,cloud.height);
	//fullcloudr.points.resize(0);

	if(counterr > 10){exit(0);}
//		cv::Mat rgb;
//		rgb.create(cloud.height,cloud.width,CV_8UC3);
//		unsigned char * rgbdata = (unsigned char *)rgb.data;
//		unsigned int nr_data = cloud.height * cloud.width;
//		for(unsigned int i = 0; i < nr_data; i++){
//			pcl::PointXYZRGB p = cloud.points[i];
//			rgbdata[3*i+0]	= p.b;
//			rgbdata[3*i+1]	= p.g;
//			rgbdata[3*i+2]	= p.r;
//		}
//		cv::namedWindow(	"rgb", cv::WINDOW_AUTOSIZE );
//		cv::imshow(			"rgb", rgb );
//		cv::waitKey(30);
//	}
	counterr++;
}

int main(int argc, char **argv){
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = boost::shared_ptr<pcl::visualization::PCLVisualizer>(new pcl::visualization::PCLVisualizer ("viewer"));
	viewer->addCoordinateSystem();
	viewer->setBackgroundColor(0.8,0.8,0.8);

	string pathr = "/velodyne_points_right";
	string pathl = "/velodyne_points_left";
	if(argc > 1){	pathr = string(argv[1]);}
	if(argc > 2){	pathl = string(argv[2]);}

	ros::init(argc, argv, "massregVelodyneNode");
	ros::NodeHandle n;
	ros::Subscriber subr = n.subscribe (pathr, 0, cloud_cb_r);
	ros::Subscriber subl = n.subscribe (pathl, 0, cloud_cb_l);
	ros::spin();

	return 0;
}
