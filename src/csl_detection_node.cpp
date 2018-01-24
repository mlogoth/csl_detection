#include <ros/ros.h>
#include <cstdlib>
#include <iostream>
#include <math.h>
#include <stdio.h>
#include <sensor_msgs/PointCloud2.h>   
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/transforms.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>

using namespace std;

pcl::PointCloud<pcl::PointXYZRGB>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
bool saveCloud = false;
unsigned int filesSaved = 0;


void
keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event,
					  void* nothing)
{
	if (event.getKeySym() == "space" && event.keyDown()){
		saveCloud = true;
	}
	else
	        saveCloud = false;
	
}

 void cloud_cb(const boost::shared_ptr<const sensor_msgs::PointCloud2>& input){

    //Get the Point Cloud
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*input,pcl_pc2);
    pcl::fromPCLPointCloud2(pcl_pc2,*source_cloud);
    
     
    //Transform the Cloud
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    //translation (none)
    transform.translation() << 0.0, 0.0, 0.0;
    //rotation matrix -PI radians arround X axis
    transform.rotate (Eigen::AngleAxisf (-M_PI, Eigen::Vector3f::UnitX()));
    // Executing the transformation
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
    pcl::transformPointCloud (*source_cloud, *transformed_cloud, transform);
    
    viewer.registerKeyboardCallback(keyboardEventOccurred);
    viewer.showCloud (transformed_cloud);
    
    if (saveCloud)
	{
		stringstream stream;
		//stream << "inputCloud" << filesSaved << ".ply";
		stream << "inputCloud" << filesSaved << ".pcd";
		string filename = stream.str();
		//if (pcl::io::savePLYFile(filename, *transformed_cloud, true) == 0)
		if (pcl::io::savePCDFile(filename, *transformed_cloud, true) == 0)
		{
			filesSaved++;
			cout << "Saved " << filename << "." << endl;
		}
		else PCL_ERROR("Problem saving %s.\n", filename.c_str());
 
		saveCloud = false;
	}
}



int main (int argc, char **argv){

    ros::init (argc, argv, "csl_detection_node");
    ros::NodeHandle nh;
    ros::Rate loop_rate( 10.0 );
    ros::Subscriber sub = nh.subscribe("/kinect2/sd/points", 1, cloud_cb);
    
    while (ros::ok()){
        
	
	ros::spinOnce();
	loop_rate.sleep();
     }
 	
     return 0;
}







