
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


#include <iostream>

#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

#include <string.h>

using namespace std;

boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

int counterr = 0;
int counterl = 0;
std::string path = "./";

//From crossproduct
void getNormal(float & nx, float & ny, float & nz, float x1, float y1, float z1,float x2, float y2, float z2,float x3, float y3, float z3){
	float u1 = x1-x2;
	float u2 = y1-y2;
	float u3 = z1-z2;
	//	float unorm = sqrt(u1*u1+u2*u2+u3*u3);
	//	u1 /= unorm;
	//	u2 /= unorm;
	//	u3 /= unorm;

	float v1 = x1-x3;
	float v2 = y1-y3;
	float v3 = z1-z3;
	//	float vnorm = sqrt(v1*v1+v2*v2+v3*v3);
	//	v1 /= vnorm;
	//	v2 /= vnorm;
	//	v3 /= vnorm;


	//Corssprod u x v
	nx = u2*v3-u3*v2;
	ny = u3*v1-u1*v3;
	nz = u1*v2-u2*v1;

	float nnorm = sqrt(nx*nx+ny*ny+nz*nz);
	nx /= nnorm;
	ny /= nnorm;
	nz /= nnorm;

	//printf("%f %f %f\n",nx,ny,nz);
	//Flip direction if point + normal further away than point
	if(((x1+nx)*(x1+nx) + (y1+ny)*(y1+ny) + (z1+nz)*(z1+nz)) > (x1*x1 + y1*y1 + z1*z1)){
		nx = -nx;
		ny = -ny;
		nz = -nz;
	}


}

double scoren (Eigen::Vector3d & a, Eigen::Vector3d & b){
	double v = a.dot(b);
	if(std::isnan(v)){return 0;}
	return v*v;
}

unsigned int text_id = 0;
bool gonext = false;
void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event,  void* viewer_void) {
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);
	if (event.getKeySym () == "n" && event.keyDown ()){
		gonext = true;
		std::cout << "n was pressed => removing all text" << std::endl;
	}
}

void fillNormals(pcl::PointCloud<pcl::PointXYZRGBNormal> & cloud, int normaltype = 0){
	viewer->registerKeyboardCallback (keyboardEventOccurred, (void*)&viewer);
	int stepw = 2*16;
	/*
	unsigned int nrp = cloud.points.size();
	//for(int i = 0; i < nrp; i++){
	while(true){
		int i = rand()%nrp;
		viewer->removeAllShapes();
		viewer->addSphere<pcl::PointXYZRGBNormal>(cloud.points[i],0.1, 0,0,255, "S1");
		viewer->addSphere<pcl::PointXYZRGBNormal>(cloud.points[i+2],0.1, 255,0,255, "S2");
		viewer->spin();

//		int jstart = std::max(int(0),i-16);
//		int jstop = std::min(int(nrp),i+16);
//		printf("%i -> %i %i\n",i,jstart,jstop);
//		for(int j = jstart; j < jstop; j++){
//			printf("%i %i\n",i,j);
//			viewer->removeAllShapes();
//			viewer->addSphere<pcl::PointXYZRGBNormal>(cloud.points[i],0.1, 0,0,255, "S1");
//			viewer->addSphere<pcl::PointXYZRGBNormal>(cloud.points[j],0.1, 255,0,255, "S2");
//			viewer->spin();
//			if(gonext){gonext = false;j = jstop;}
//		}
	}
	//}
	*/


	unsigned int nrp = cloud.points.size();
	for(int i = 0; i < cloud.points.size(); i++){
		pcl::PointXYZRGBNormal & np = cloud.points[i];

		pcl::PointXYZRGBNormal & right = cloud.points[(i+stepw)%nrp];
		pcl::PointXYZRGBNormal & left = cloud.points[(nrp+i-stepw)%nrp];
		pcl::PointXYZRGBNormal up;
		if(i%16 != 0){up = cloud.points[i-1];}

		pcl::PointXYZRGBNormal down;
		if(i%16 != 15){down = cloud.points[i+1];}

		Eigen::Vector3d npv (np.x,np.y,np.z);
		Eigen::Vector3d rv (right.x,right.y,right.z);
		Eigen::Vector3d uv (up.x,up.y,up.z);
		Eigen::Vector3d lv (left.x,left.y,left.z);
		Eigen::Vector3d dv (down.x,down.y,down.z);

		Eigen::Vector3d n1 = (npv-rv).cross(npv-uv);
		Eigen::Vector3d n2 = (npv-rv).cross(npv-dv);
		Eigen::Vector3d n3 = (npv-lv).cross(npv-uv);
		Eigen::Vector3d n4 = (npv-lv).cross(npv-dv);

		n1.normalize();
		n2.normalize();
		n3.normalize();
		n4.normalize();

		double s1 = scoren(n1,n1)+scoren(n1,n2)+scoren(n1,n3)+scoren(n1,n4);
		double s2 = scoren(n2,n1)+scoren(n2,n2)+scoren(n2,n3)+scoren(n2,n4);
		double s3 = scoren(n3,n1)+scoren(n3,n2)+scoren(n3,n3)+scoren(n3,n4);
		double s4 = scoren(n4,n1)+scoren(n4,n2)+scoren(n4,n3)+scoren(n4,n4);

		Eigen::Vector3d n = n1;
		double score = s1;
		if(s2 > score){score = s2; n = n2;}
		if(s3 > score){score = s3; n = n3;}
		if(s4 > score){score = s4; n = n4;}
		//		float nx1,ny1,nz1;
		//		getNormal(nx1,ny1,nz1,np.x,np.y,np.z,right.x,right.y,right.z,up.x,up.y,up.z);
		//		float nx2,ny2,nz2;
		//		getNormal(nx2,ny2,nz2,np.x,np.y,np.z,right.x,right.y,right.z,down.x,down.y,down.z);
		//		float nx3,ny3,nz3;
		//		getNormal(nx3,ny3,nz3,np.x,np.y,np.z,left.x,left.y,left.z,up.x,up.y,up.z);
		//		float nx4,ny4,nz4;
		//		getNormal(nx4,ny4,nz4,np.x,np.y,np.z,left.x,left.y,left.z,down.x,down.y,down.z);


		if((npv+n).norm() > npv.norm()){
			n = -n;
		}

		np.normal_x = n(0);
		np.normal_y = n(1);
		np.normal_z = n(2);


		//		if(normaltype == 0){
		//			np.normal_x = n1(0);//nx1;
		//			np.normal_y = n1(1);//ny1;
		//			np.normal_z = n1(2);//nz1;
		//		}
		//		if(normaltype == 1){
		//			np.normal_x = n2(0);//nx1;
		//			np.normal_y = n2(1);//ny1;
		//			np.normal_z = n2(2);//nz1;
		//		}
		//		if(normaltype == 2){
		//			np.normal_x = n3(0);//nx1;
		//			np.normal_y = n3(1);//ny1;
		//			np.normal_z = n3(2);//nz1;
		//		}
		//		if(normaltype == 3){
		//			np.normal_x = n4(0);//nx1;
		//			np.normal_y = n4(1);//ny1;
		//			np.normal_z = n4(2);//nz1;
		//		}
		//		if(i%17 == 0){
		//			viewer->removeAllShapes();
		//			viewer->addSphere<pcl::PointXYZRGBNormal>(np,0.1, 0,0,255, "S1");
		//			viewer->addSphere<pcl::PointXYZRGBNormal>(right,0.1, 255,0,255, "S2");
		//			viewer->addSphere<pcl::PointXYZRGBNormal>(left,0.1, 255,0,0, "S3");
		//			viewer->addSphere<pcl::PointXYZRGBNormal>(up,0.1, 0,255,255, "S4");
		//			viewer->addSphere<pcl::PointXYZRGBNormal>(down,0.1, 255,255,255, "S5");
		//			pcl::PointXYZRGBNormal tmp;
		//			tmp.x = np.x+np.normal_x;
		//			tmp.y = np.y+np.normal_y;
		//			tmp.z = np.z+np.normal_z;
		//			viewer->addLine<pcl::PointXYZRGBNormal> (np,right,	255,255,0,"L1");
		//			viewer->addLine<pcl::PointXYZRGBNormal> (np,left,	255,0,0,"L2");
		//			viewer->addLine<pcl::PointXYZRGBNormal> (np,up,		0,255,255,"L3");
		//			viewer->addLine<pcl::PointXYZRGBNormal> (np,down,	255,255,255,"L4");
		//			viewer->addLine<pcl::PointXYZRGBNormal> (np,tmp,	255,0,255,"L5");
		//			viewer->spin();
		//		}

	}

}

pcl::PointCloud<pcl::PointXYZRGBNormal> getCloudWithNormals(pcl::PointCloud<pcl::PointXYZRGB> & cloud, int normaltype = 0){
	pcl::PointCloud<pcl::PointXYZRGBNormal> normalcloud;
	normalcloud.points.resize(cloud.points.size());
	for(int i = 0; i < cloud.points.size(); i++){
		pcl::PointXYZRGB & op = cloud.points[i];
		pcl::PointXYZRGBNormal & np = normalcloud.points[i];
		np.r = 0;//op.r;
		np.g = 255;//op.g;
		np.b = 0;//op.b;
		np.x = op.x;
		np.y = op.y;
		np.z = op.z;
	}

	//	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	//	*cloud_ptr = normalcloud;
	//	viewer->removeAllPointClouds();
	//	viewer->addPointCloud<pcl::PointXYZRGBNormal> (cloud_ptr, pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBNormal>(cloud_ptr), "cloud");
	//	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud");
	//	//viewer->addPointCloudNormals<pcl::PointXYZRGBNormal> (cloud_ptr, 17, 1.5, "normals");
	//	viewer->spin();
	fillNormals(normalcloud,normaltype);
	return normalcloud;
}

void  cloud_cb_l(const sensor_msgs::PointCloud2ConstPtr& input){
	ROS_INFO("l pointcloud in %i",counterl);
}

pcl::PointCloud<pcl::PointXYZRGB> prevr;
std::vector< pcl::PointCloud<pcl::PointXYZRGB> > all_clouds;
void  cloud_cb_r(const sensor_msgs::PointCloud2ConstPtr& input){
	ROS_INFO("r pointcloud in %i",counterr);

	pcl::PointCloud<pcl::PointXYZRGB> tmpcloud;
	pcl::fromROSMsg (*input, tmpcloud);

	pcl::PointCloud<pcl::PointXYZRGB> cloud;
	cloud.resize(tmpcloud.points.size()/2);
	for(int i = 0; i < tmpcloud.points.size(); i+=2){
		cloud.points[i/2] = tmpcloud.points[i];
	}


	int r [16];
	int g [16];
	int b [16];
	for(int h = 0; h < 16; h++){
		r[h] = rand() % 256;
		g[h] = rand() % 256;
		b[h] = rand() % 256;
	}

	cloud.height = 16;
	cloud.width = cloud.points.size()/cloud.height;

	for(int i = 0; i < cloud.points.size(); i++){
		cloud.points[i].r = r[i%16];
		cloud.points[i].g = g[i%16];
		cloud.points[i].b = b[i%16];
	}

	char buf[1024];
	sprintf(buf,"%s/right_%.10i.pcd",path.c_str(),counterr);
	//pcl::io::savePCDFileBinary (string(buf), cloud);
	printf("nr points : %i resolution %i %i\n",cloud.points.size(),cloud.width,cloud.height);

	if(counterr % 2 == 1){
		pcl::PointCloud<pcl::PointXYZRGB> fullcloud = prevr+cloud;
		for(int i = 0; i < fullcloud.points.size(); i++){
			fullcloud.points[i].r = r[i%16];
			fullcloud.points[i].g = g[i%16];
			fullcloud.points[i].b = b[i%16];
		}
		sprintf(buf,"%s/fullright_%.10i.pcd",path.c_str(),counterr);
		//pcl::io::savePCDFileBinary (string(buf), fullcloud);


		pcl::PointCloud<pcl::PointXYZRGBNormal> fullcloudnormals = getCloudWithNormals(fullcloud,0);

		//all_clouds.push_back(fullcloudnormals);

		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
		*cloud_ptr = fullcloudnormals;
		viewer->removeAllPointClouds();
		viewer->addPointCloud<pcl::PointXYZRGBNormal> (cloud_ptr, pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBNormal>(cloud_ptr), "cloud");
		viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud");
		viewer->addPointCloudNormals<pcl::PointXYZRGBNormal> (cloud_ptr, 17, 0.5, "normals");
		viewer->spinOnce();

		sprintf(buf,"%s/normalsfullright_%.10i.pcd",path.c_str(),counterr);
		pcl::io::savePCDFileBinary (string(buf), fullcloudnormals);
	}
	//

	prevr = cloud;
	//if(counterr > 10){exit(0);}
	counterr++;
}

int main(int argc, char **argv){
	viewer = boost::shared_ptr<pcl::visualization::PCLVisualizer>(new pcl::visualization::PCLVisualizer ("viewer"));
	viewer->addCoordinateSystem();
	viewer->setBackgroundColor(0.8,0.8,0.8);
	viewer->initCameraParameters ();

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
