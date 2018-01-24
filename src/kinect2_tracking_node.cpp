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
typedef pcl::Normal NormalType;
typedef pcl::ReferenceFrame RFType;
typedef pcl::SHOT352 DescriptorType;

using namespace std;

string model_filename_ = "/home/michalis/Downloads/coke.ply" ; //"/home/karrasg/Downloads/libfreenect2pclgrabber/build/test_cloud.ply";
//string model_filename_ = "/home/karrasg/Downloads/libfreenect2pclgrabber/build/test_cloud2.ply" ;

//Algorithm params
bool show_keypoints_ (false);
bool show_correspondences_ (false);
bool use_cloud_resolution_ (false);
bool use_hough_ (true);
float model_ss_ (0.01f);
float scene_ss_ (0.03f);
float rf_rad_ (0.015f);
float descr_rad_ (0.02f);
float cg_size_ (0.001f);
float cg_thresh_ (.5f);

int main (int argc, char **argv){

    ros::init (argc, argv, "kinect2_tracking_node");
    ros::NodeHandle nh;
    ros::Rate loop_rate(30);
    
    processor freenectprocessor = CUDA;
    
    
    pcl::PointCloud<PointType>::Ptr model (new pcl::PointCloud<PointType> ());
    pcl::PointCloud<PointType>::Ptr model_keypoints (new pcl::PointCloud<PointType> ());
    pcl::PointCloud<PointType>::Ptr scene (new pcl::PointCloud<PointType> ());
    pcl::PointCloud<PointType>::Ptr scene_keypoints (new pcl::PointCloud<PointType> ());
    pcl::PointCloud<NormalType>::Ptr model_normals (new pcl::PointCloud<NormalType> ());
    pcl::PointCloud<NormalType>::Ptr scene_normals (new pcl::PointCloud<NormalType> ());
    pcl::PointCloud<DescriptorType>::Ptr model_descriptors (new pcl::PointCloud<DescriptorType> ());
    pcl::PointCloud<DescriptorType>::Ptr scene_descriptors (new pcl::PointCloud<DescriptorType> ());
    
    
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
    
    
    
    //  Compute Normals
    pcl::NormalEstimationOMP<PointType, NormalType> norm_est;
    norm_est.setKSearch (10);
    norm_est.setInputCloud (model);//Model
    norm_est.compute (*model_normals);
    
    //  Downsample Model Cloud to Extract keypoints
    pcl::UniformSampling<PointType> uniform_sampling;
    uniform_sampling.setInputCloud (model);
    uniform_sampling.setRadiusSearch (model_ss_);
    pcl::PointCloud<int> keypointIndices1;
    uniform_sampling.compute(keypointIndices1);
    pcl::copyPointCloud(*model, keypointIndices1.points, *model_keypoints);
    //std::cout << "Model total points: " << model->size () << "; Selected Keypoints: " << model_keypoints->size () << std::endl; 
    
  
   //Compute Descriptor for keypoints
   pcl::SHOTEstimationOMP<PointType, NormalType, DescriptorType> descr_est;
   descr_est.setRadiusSearch (descr_rad_);
   //Model first
   descr_est.setInputCloud (model_keypoints);
   descr_est.setInputNormals (model_normals);
   descr_est.setSearchSurface (model);
   descr_est.compute (*model_descriptors);


    while (ros::ok()){
    
    	viewer->spinOnce ();
    	std::chrono::high_resolution_clock::time_point tnow = std::chrono::high_resolution_clock::now();

    	scene = k2g.getCloud();

    	std::chrono::high_resolution_clock::time_point tpost = std::chrono::high_resolution_clock::now();
    	//std::cout << "delta " << std::chrono::duration_cast<std::chrono::duration<double>>(tpost-tnow).count() * 1000 << std::endl;
    	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(scene);
    	viewer->updatePointCloud<pcl::PointXYZRGB> (scene, rgb, "sample cloud");    	
	
	
	//Compute Normals Scene
    	norm_est.setInputCloud (scene);
  	norm_est.compute (*scene_normals);
  	
  	//Downsample scene to extract keypoints
  	uniform_sampling.setInputCloud (scene);
        uniform_sampling.setRadiusSearch (scene_ss_);
        pcl::PointCloud<int> keypointIndices2;
        uniform_sampling.compute(keypointIndices2);
        pcl::copyPointCloud(*scene, keypointIndices2.points, *scene_keypoints);
        //std::cout << "Scene total points: " << scene->size () << "; Selected Keypoints: " << scene_keypoints->size () << std::endl; 
        
        //Compute Descriptors for scene
        descr_est.setInputCloud (scene_keypoints);
        descr_est.setInputNormals (scene_normals);
        descr_est.setSearchSurface (scene);
        descr_est.compute (*scene_descriptors);
        
        
        //
        //Find Model-Scene Correspondences with KdTree
  	pcl::CorrespondencesPtr model_scene_corrs (new pcl::Correspondences ());

  	pcl::KdTreeFLANN<DescriptorType> match_search;
  	match_search.setInputCloud (model_descriptors);

  	//  For each scene keypoint descriptor, find nearest neighbor into the model keypoints descriptor cloud and add it to the correspondences vector.
  	for (size_t i = 0; i < scene_descriptors->size (); ++i)
  	{
    		std::vector<int> neigh_indices (1);
    		std::vector<float> neigh_sqr_dists (1);
    		if (!pcl_isfinite (scene_descriptors->at (i).descriptor[0])) //skipping NaNs
    		{
      			continue;
    		}
    		int found_neighs = match_search.nearestKSearch (scene_descriptors->at (i), 1, neigh_indices, neigh_sqr_dists);
    		if(found_neighs == 1 && neigh_sqr_dists[0] < 0.25f) //  add match only if the squared descriptor distance is less than 0.25 (SHOT descriptor distances are between 0 and 1 by design)
    		{
      			pcl::Correspondence corr (neigh_indices[0], static_cast<int> (i), neigh_sqr_dists[0]);
      			model_scene_corrs->push_back (corr);
    		}
  	}
  	std::cout << "Correspondences found: " << model_scene_corrs->size () << std::endl;
  	
  	
  	//
  	//  Actual Clustering
       	std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations;
  	std::vector<pcl::Correspondences> clustered_corrs;

  	//  Using Hough3D

    	//
    	//  Compute (Keypoints) Reference Frames only for Hough
    	//
    	pcl::PointCloud<RFType>::Ptr model_rf (new pcl::PointCloud<RFType> ());
    	pcl::PointCloud<RFType>::Ptr scene_rf (new pcl::PointCloud<RFType> ());

    	pcl::BOARDLocalReferenceFrameEstimation<PointType, NormalType, RFType> rf_est;
    	rf_est.setFindHoles (true);
    	rf_est.setRadiusSearch (rf_rad_);

    	rf_est.setInputCloud (model_keypoints);
    	rf_est.setInputNormals (model_normals);
    	rf_est.setSearchSurface (model);
    	rf_est.compute (*model_rf);

    	rf_est.setInputCloud (scene_keypoints);
    	rf_est.setInputNormals (scene_normals);
    	rf_est.setSearchSurface (scene);
    	rf_est.compute (*scene_rf);

    	//  Clustering
    	pcl::Hough3DGrouping<PointType, PointType, RFType, RFType> clusterer;
    	clusterer.setHoughBinSize (cg_size_);
    	clusterer.setHoughThreshold (cg_thresh_);
    	clusterer.setUseInterpolation (true);
    	clusterer.setUseDistanceWeight (false);

    	clusterer.setInputCloud (model_keypoints);
    	clusterer.setInputRf (model_rf);
    	clusterer.setSceneCloud (scene_keypoints);
    	clusterer.setSceneRf (scene_rf);
    	clusterer.setModelSceneCorrespondences (model_scene_corrs);

    	//clusterer.cluster (clustered_corrs);
    	clusterer.recognize (rototranslations, clustered_corrs);
    	
    	//
  	//  Output results
  	//
  	std::cout << "Model instances found: " << rototranslations.size () << std::endl;
  	for (size_t i = 0; i < rototranslations.size (); ++i)
 	{
    		std::cout << "\n    Instance " << i + 1 << ":" << std::endl;
    		std::cout << "        Correspondences belonging to this instance: " << clustered_corrs[i].size () << std::endl;

    		// Print the rotation matrix and translation vector
    		Eigen::Matrix3f rotation = rototranslations[i].block<3,3>(0, 0);
    		Eigen::Vector3f translation = rototranslations[i].block<3,1>(0, 3);

    		printf ("\n");
    		printf ("            | %6.3f %6.3f %6.3f | \n", rotation (0,0), rotation (0,1), rotation (0,2));
    		printf ("        R = | %6.3f %6.3f %6.3f | \n", rotation (1,0), rotation (1,1), rotation (1,2));
    		printf ("            | %6.3f %6.3f %6.3f | \n", rotation (2,0), rotation (2,1), rotation (2,2));
    		printf ("\n");
    		printf ("        t = < %0.3f, %0.3f, %0.3f >\n", translation (0), translation (1), translation (2));
  	}
  /*
  	//
  	//  Visualization
  	//
  	pcl::visualization::PCLVisualizer viewer ("Correspondence Grouping");
  	viewer.addPointCloud (scene, "scene_cloud");

  	pcl::PointCloud<PointType>::Ptr off_scene_model (new pcl::PointCloud<PointType> ());
  	pcl::PointCloud<PointType>::Ptr off_scene_model_keypoints (new pcl::PointCloud<PointType> ());

  	if (show_correspondences_ || show_keypoints_)
  	{
    		//  We are translating the model so that it doesn't end in the middle of the scene representation
    		pcl::transformPointCloud (*model, *off_scene_model, Eigen::Vector3f (-1,0,0), Eigen::Quaternionf (1, 0, 0, 0));
    		pcl::transformPointCloud (*model_keypoints, *off_scene_model_keypoints, Eigen::Vector3f (-1,0,0), Eigen::Quaternionf (1, 0, 0, 0));

    		pcl::visualization::PointCloudColorHandlerCustom<PointType> off_scene_model_color_handler (off_scene_model, 255, 255, 128);
    		viewer.addPointCloud (off_scene_model, off_scene_model_color_handler, "off_scene_model");
  	}

  	if (show_keypoints_)
  	{
    		pcl::visualization::PointCloudColorHandlerCustom<PointType> scene_keypoints_color_handler (scene_keypoints, 0, 0, 255);
    		viewer.addPointCloud (scene_keypoints, scene_keypoints_color_handler, "scene_keypoints");
    		viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "scene_keypoints");

    		pcl::visualization::PointCloudColorHandlerCustom<PointType> off_scene_model_keypoints_color_handler (off_scene_model_keypoints, 0, 0, 255);
    		viewer.addPointCloud (off_scene_model_keypoints, off_scene_model_keypoints_color_handler, "off_scene_model_keypoints");
    		viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "off_scene_model_keypoints");
  	}

  	for (size_t i = 0; i < rototranslations.size (); ++i)
  	{
    		pcl::PointCloud<PointType>::Ptr rotated_model (new pcl::PointCloud<PointType> ());
    		pcl::transformPointCloud (*model, *rotated_model, rototranslations[i]);

    		std::stringstream ss_cloud;
    		ss_cloud << "instance" << i;

    		pcl::visualization::PointCloudColorHandlerCustom<PointType> rotated_model_color_handler (rotated_model, 255, 0, 0);
    		viewer.addPointCloud (rotated_model, rotated_model_color_handler, ss_cloud.str ());

    		if (show_correspondences_)
    		{
      			for (size_t j = 0; j < clustered_corrs[i].size (); ++j)
      			{
        			std::stringstream ss_line;
        			ss_line << "correspondence_line" << i << "_" << j;
        			PointType& model_point = off_scene_model_keypoints->at (clustered_corrs[i][j].index_query);
        			PointType& scene_point = scene_keypoints->at (clustered_corrs[i][j].index_match);

        			//  We are drawing a line for each pair of clustered correspondences found between the model and the scene
        			viewer.addLine<PointType, PointType> (model_point, scene_point, 0, 255, 0, ss_line.str ());
      			}
    		}
  	}*/
  	
	
    }
 	
 	
    k2g.shutDown();
  
    return 0;
}
