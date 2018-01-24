#include <ros/ros.h>
#include <cstdlib>
#include <chrono>
#include <iostream>
#include <math.h>
#include <stdio.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/console/parse.h>
#include <pcl/common/time.h>
#include <pcl/common/centroid.h>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

#include <pcl/search/pcl_search.h>
#include <pcl/common/transforms.h>

#include <boost/format.hpp>

#include <pcl/tracking/tracking.h>
#include <pcl/tracking/particle_filter.h>
#include <pcl/tracking/kld_adaptive_particle_filter_omp.h>
#include <pcl/tracking/particle_filter_omp.h>
#include <pcl/tracking/coherence.h>
#include <pcl/tracking/distance_coherence.h>
#include <pcl/tracking/hsv_color_coherence.h>
#include <pcl/tracking/approx_nearest_pair_point_cloud_coherence.h>
#include <pcl/tracking/nearest_pair_point_cloud_coherence.h>

#define WITH_PCL
#include <csl_detection/k2g.h>

using namespace pcl::tracking;

typedef pcl::PointXYZRGB PointType;
typedef pcl::PointXYZRGB RefPointType;

typedef ParticleXYZRPY ParticleT;
typedef pcl::PointCloud<pcl::PointXYZRGBA> Cloud;
typedef Cloud::Ptr CloudPtr;
typedef Cloud::ConstPtr CloudConstPtr;
typedef ParticleFilterTracker<RefPointType, ParticleT> ParticleFilter;

using namespace std;

CloudPtr cloud_pass_;
CloudPtr cloud_pass_downsampled_;
CloudPtr target_cloud;

boost::mutex mtx_;
boost::shared_ptr<ParticleFilter> tracker_;
bool new_cloud_;
double downsampling_grid_size_;
int counter;


//Filter along a specified dimension
void filterPassThrough (const CloudConstPtr &cloud, Cloud &result)
{
  pcl::PassThrough<pcl::PointXYZRGBA> pass;
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.0, 10.0);
  pass.setKeepOrganized (false);
  pass.setInputCloud (cloud);
  pass.filter (result);
}


void gridSampleApprox (const CloudConstPtr &cloud, Cloud &result, double leaf_size)
{
  pcl::ApproximateVoxelGrid<pcl::PointXYZRGBA> grid;
  grid.setLeafSize (static_cast<float> (leaf_size), static_cast<float> (leaf_size), static_cast<float> (leaf_size));
  grid.setInputCloud (cloud);
  grid.filter (result);
}

void
cloud_cb (const CloudPtr &cloud)
{
  boost::mutex::scoped_lock lock (mtx_);
  cloud_pass_.reset (new Cloud);
  cloud_pass_downsampled_.reset (new Cloud);
  filterPassThrough (cloud, *cloud_pass_);
  gridSampleApprox (cloud_pass_, *cloud_pass_downsampled_, downsampling_grid_size_);

  if(counter < 10){
	counter++;
  }else{
  	//Track the object
  	CloudConstPtr const_cloud_pass_downsampled_ (cloud_pass_downsampled_);
	tracker_->setInputCloud (&const_cloud_pass_downsampled_);
	tracker_->compute ();
	new_cloud_ = true;
  }
}


int main (int argc, char **argv){

    ros::init (argc, argv, "kinect2_tracking_node");
    ros::NodeHandle nh;
    ros::Rate loop_rate(30);
    
     if (argc < 2)
    {
      PCL_WARN("Please set device_id pcd_filename(e.g. $ %s '#1' sample.pcd)\n", argv[0]);
      exit (1);
    }

  //read pcd file
  target_cloud.reset(new Cloud());
  if(pcl::io::loadPCDFile (argv[2], *target_cloud) == -1){
    std::cout << "pcd file not found" << std::endl;
    exit(-1);
  }
  
	// TODO: Remove ()
    pcl::PointCloud<PointType>::Ptr scene (new pcl::PointCloud<PointType> ());

    
    // Freenect
    processor freenectprocessor = CPU;
    K2G k2g(freenectprocessor);

    // Viewer
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(scene);
    viewer->addPointCloud<pcl::PointXYZRGB>(scene, rgb, "sample cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
    
    scene = k2g.getCloud();//Get once and set camera axes
    scene->sensor_orientation_.w() = 1.0;
    scene->sensor_orientation_.x() = 0.0;
    scene->sensor_orientation_.y() = 0.0;
    scene->sensor_orientation_.z() = 0.0;
    
    
  std::string device_id = std::string (argv[1]);  

  counter = 0;

  //Set parameters
  new_cloud_  = false;
  downsampling_grid_size_ =  0.002;

  std::vector<double> default_step_covariance = std::vector<double> (6, 0.015 * 0.015);
  default_step_covariance[3] *= 40.0;
  default_step_covariance[4] *= 40.0;
  default_step_covariance[5] *= 40.0;

  std::vector<double> initial_noise_covariance = std::vector<double> (6, 0.00001);
  std::vector<double> default_initial_mean = std::vector<double> (6, 0.0);

  boost::shared_ptr<KLDAdaptiveParticleFilterOMPTracker<RefPointType, ParticleT> > tracker
    (new KLDAdaptiveParticleFilterOMPTracker<RefPointType, ParticleT> (8));

  ParticleT bin_size;
  bin_size.x = 0.1f;
  bin_size.y = 0.1f;
  bin_size.z = 0.1f;
  bin_size.roll = 0.1f;
  bin_size.pitch = 0.1f;
  bin_size.yaw = 0.1f;


  //Set all parameters for  KLDAdaptiveParticleFilterOMPTracker
  tracker->setMaximumParticleNum (1000);
  tracker->setDelta (0.99);
  tracker->setEpsilon (0.2);
  tracker->setBinSize (bin_size);

  //Set all parameters for  ParticleFilter
  tracker_ = tracker;
  tracker_->setTrans (Eigen::Affine3f::Identity ());
  tracker_->setStepNoiseCovariance (default_step_covariance);
  tracker_->setInitialNoiseCovariance (initial_noise_covariance);
  tracker_->setInitialNoiseMean (default_initial_mean);
  tracker_->setIterationNum (1);
  tracker_->setParticleNum (600);
  tracker_->setResampleLikelihoodThr(0.00);
  tracker_->setUseNormal (false);


  //Setup coherence object for tracking
  ApproxNearestPairPointCloudCoherence<RefPointType>::Ptr coherence = ApproxNearestPairPointCloudCoherence<RefPointType>::Ptr
    (new ApproxNearestPairPointCloudCoherence<RefPointType> ());
    
  boost::shared_ptr<DistanceCoherence<RefPointType> > distance_coherence
    = boost::shared_ptr<DistanceCoherence<RefPointType> > (new DistanceCoherence<RefPointType> ());
  coherence->addPointCoherence (distance_coherence);

  boost::shared_ptr<pcl::search::Octree<RefPointType> > search (new pcl::search::Octree<RefPointType> (0.01));
  coherence->setSearchMethod (search);
  coherence->setMaximumDistance (0.01);

  tracker_->setCloudCoherence (coherence);

  //prepare the model of tracker's target
  Eigen::Vector4f c;
  Eigen::Affine3f trans = Eigen::Affine3f::Identity ();
  CloudPtr transed_ref (new Cloud);
  CloudPtr transed_ref_downsampled (new Cloud);

  pcl::compute3DCentroid<RefPointType> (*target_cloud, c);
  trans.translation ().matrix () = Eigen::Vector3f (c[0], c[1], c[2]);
  pcl::transformPointCloud<RefPointType> (*target_cloud, *transed_ref, trans.inverse());
  gridSampleApprox (transed_ref, *transed_ref_downsampled, downsampling_grid_size_);

  //set reference model and trans
  tracker_->setReferenceCloud (transed_ref_downsampled);
  tracker_->setTrans (trans);

    while (ros::ok()){
    
    	viewer->spinOnce ();

    	scene = k2g.getCloud();
    	//cloud_cb(scene);
    	
    	cloud_pass_.reset (new Cloud);
  		cloud_pass_downsampled_.reset (new Cloud);
  		filterPassThrough (scene, *cloud_pass_);
  		gridSampleApprox (cloud_pass_, *cloud_pass_downsampled_, downsampling_grid_size_);

  		if(counter < 10){
			counter++;
  		}else{
  		//Track the object
		//TODO
		//tracker_->setInputCloud (cloud_pass_downsampled_);
		tracker_->setInputCloud (cloud);
		tracker_->compute ();
		new_cloud_ = true;
  		}
  		
    	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(scene);
    	viewer->updatePointCloud<pcl::PointXYZRGB> (scene, rgb, "sample cloud");
		ros::spinOnce();
		
    }
 	
 	
    k2g.shutDown();
  
    return 0;
}
