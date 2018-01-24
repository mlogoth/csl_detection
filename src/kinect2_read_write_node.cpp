#include <ros/ros.h>
#include <cstdlib>
#include <chrono>
#include <iostream>
#include <math.h>
#include <stdio.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/correspondence.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/board.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>

#define WITH_PCL
#include <csl_detection/k2g.h>

typedef pcl::PointXYZRGB PointType;

using namespace std;

string model_filename_ = "/home/karrasg/Downloads/libfreenect2pclgrabber/build/test_cloud.ply";



int main (int argc, char **argv){

    ros::init (argc, argv, "kinect2_read_write_node");
    ros::NodeHandle nh;
    ros::Rate loop_rate(30);
    
    processor freenectprocessor = CPU;
    
    
    pcl::PointCloud<PointType>::Ptr scene (new pcl::PointCloud<PointType> ());
    pcl::PointCloud<PointType>::Ptr model (new pcl::PointCloud<PointType> ());
    
    
    K2G k2g(freenectprocessor);
    
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer ("3D Viewer"));
    
    
    scene = k2g.getCloud();//Get once and set camera axes
    scene->sensor_orientation_.w() = 0.0;
    scene->sensor_orientation_.x() = 1.0;
    scene->sensor_orientation_.y() = 0.0;
    scene->sensor_orientation_.z() = 0.0;
    
    viewer->setBackgroundColor (0, 0, 0);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(scene);
    viewer->addPointCloud<pcl::PointXYZRGB>(scene, rgb, "sample cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
    
    
    
    
    //Reading Model From File
    if (pcl::io::loadPLYFile (model_filename_, *model) < 0){
    	std::cout << "Error loading model cloud." << std::endl;
    	return (-1);
    }
    
    
    //Create Cube Model
    //generate cloud (CUBE)
    /*int side = 10 + 1;
    model->points.resize (pow (side, 3));
    model->width = model->points.size ();
    model->height = 1;

    int p = 0;
    for (size_t i = 0; i < side; i++)
      for (size_t j = 0; j < side; j++)
        for (size_t k = 0; k < side; k++, p++)
         {
          model->points[p].getVector3fMap () = Eigen::Vector3f (i, j, k);
         } */
    
  
    int i = 0;
    while (i < 500000){
    
        i++;
    	viewer->spinOnce ();
    	std::chrono::high_resolution_clock::time_point tnow = std::chrono::high_resolution_clock::now();

    	//scene = k2g.getCloud();

    	std::chrono::high_resolution_clock::time_point tpost = std::chrono::high_resolution_clock::now();
    	//std::cout << "delta " << std::chrono::duration_cast<std::chrono::duration<double>>(tpost-tnow).count() * 1000 << std::endl;
    	//pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(scene);
    	//viewer->updatePointCloud<pcl::PointXYZRGB> (scene, rgb, "sample cloud");    
    	
    	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(model);
    	viewer->updatePointCloud<pcl::PointXYZRGB> (model, rgb, "sample cloud");    
    	
    	
    	//pcl::io::savePCDFileASCII ("test_pcd.pcd", *scene);
        //std::cerr << "Saved " << std::endl;		
    	}
 	
 	
    k2g.shutDown();
  
    return 0;
}

