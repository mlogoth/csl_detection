#include "ros/ros.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/Pose.h"
#include <tf_conversions/tf_eigen.h>
#include "tf/transform_datatypes.h"
#include <eigen_conversions/eigen_msg.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni2_grabber.h>
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

#include <Eigen/Core>
#include <Eigen/Geometry>



using namespace pcl::tracking;
using namespace std;

string ref_pcd_file = "/home/mike/Downloads/coke_csl.pcd";

typedef pcl::PointXYZRGBA RefPointType;
typedef ParticleXYZRPY ParticleT;
typedef pcl::PointCloud<pcl::PointXYZRGBA> Cloud;
typedef Cloud::Ptr CloudPtr;
typedef Cloud::ConstPtr CloudConstPtr;
typedef ParticleFilterTracker<RefPointType, ParticleT> ParticleFilter;

CloudPtr cloud_pass_;
CloudPtr cloud_pass_downsampled_;
CloudPtr cloud_pass_downsampled_filtered;
CloudPtr target_cloud;

boost::mutex mtx_;
boost::shared_ptr<ParticleFilter> tracker_;
bool new_cloud_;
double downsampling_grid_size_, z_max, z_min;
int counter;

// Global Publisher
ros::Publisher result_pub;


//Filter along a specified dimension
void filterPassThrough (const CloudConstPtr &cloud, Cloud &result)
{
  pcl::PassThrough<pcl::PointXYZRGBA> pass;
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (z_min, z_max);
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


//Draw the current particles
bool
drawParticles (pcl::visualization::PCLVisualizer& viz)
{
  ParticleFilter::PointCloudStatePtr particles = tracker_->getParticles ();
  if (particles && new_cloud_)
    {
      //Set pointCloud with particle's points
      pcl::PointCloud<pcl::PointXYZ>::Ptr particle_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
      for (size_t i = 0; i < particles->points.size (); i++)
	{
	  pcl::PointXYZ point;
          
	  point.x = particles->points[i].x;
	  point.y = particles->points[i].y;
	  point.z = particles->points[i].z;
	  particle_cloud->points.push_back (point);
	}

      //Draw red particles 
      {
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red_color (particle_cloud, 250, 99, 71);

	if (!viz.updatePointCloud (particle_cloud, red_color, "particle cloud"))
	  viz.addPointCloud (particle_cloud, red_color, "particle cloud");
      }
      return true;
    }
  else
    {
      return false;
    }
}

//Draw model reference point cloud
void
drawResult (pcl::visualization::PCLVisualizer& viz)
{
  ParticleXYZRPY result = tracker_->getResult ();
  Eigen::Affine3f transformation = tracker_->toEigenMatrix (result);
  
  
  //geometry_msgs::Pose msg(new geometry_msgs::Pose) ;
  //tf::poseEigenToMsg(*transformation, *msg);
  // Publish Message
  //result_pub.publish(msg);


  cout<<"Result :\n" <<result << endl;
  //move close to camera a little for better visualization
  transformation.translation () += Eigen::Vector3f (0.0f, 0.0f, -0.005f);
  CloudPtr result_cloud (new Cloud ());
  pcl::transformPointCloud<RefPointType> (*(tracker_->getReferenceCloud ()), *result_cloud, transformation);

  //Draw blue model reference point cloud
  {
    pcl::visualization::PointCloudColorHandlerCustom<RefPointType> blue_color (result_cloud, 0, 0, 255);

    if (!viz.updatePointCloud (result_cloud, blue_color, "resultcloud"))
      viz.addPointCloud (result_cloud, blue_color, "resultcloud");
  }
}

//visualization's callback function
void
viz_cb (pcl::visualization::PCLVisualizer& viz)
{
  boost::mutex::scoped_lock lock (mtx_);
    
  if (!cloud_pass_)
    {
      boost::this_thread::sleep (boost::posix_time::seconds (1));
      return;
   }

  //Draw downsampled point cloud from sensor    
  if (new_cloud_ && cloud_pass_)
    {
      CloudPtr cloud_pass;
      cloud_pass = cloud_pass_;
      cout<<"Cloud Pass Size: "<<cloud_pass->size()<<endl;
      //pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> single_color(cloud_pass, 0, 255, 0);
      pcl::visualization::PointCloudColorHandlerRGBAField<pcl::PointXYZRGBA> rgba(cloud_pass_);
      if (!viz.updatePointCloud (cloud_pass, rgba,"cloudpass"))
	{
	  
	  viz.addPointCloud (cloud_pass,rgba,"cloudpass");
	  viz.resetCameraViewpoint ("cloudpass");
	}
      bool ret = drawParticles (viz);
      if (ret)
        drawResult (viz);
    }
  new_cloud_ = false;
}

//OpenNI Grabber's cloud Callback function
void
cloud_cb (const CloudConstPtr &cloud)
{
  boost::mutex::scoped_lock lock (mtx_);
  cloud_pass_.reset (new Cloud);
  cloud_pass_downsampled_.reset (new Cloud);
  //cloud_pass_downsampled_filtered.reset (new Cloud);
  filterPassThrough (cloud, *cloud_pass_);
  gridSampleApprox (cloud_pass_, *cloud_pass_downsampled_, downsampling_grid_size_);
  
  // TODO Add Removing Nan Procedure
  
  //std::vector<int> indices;
  //pcl::removeNaNFromPointCloud(*cloud_pass_downsampled_, *cloud_pass_downsampled_filtered, indices);
  
  cout <<"**************************************************"<<endl;
  cout << "--- Point Cloud Filtered: PassThrough + GridSampleApprox + RemoveNaN ---"<<endl;
  cout << "Size: "<< cloud_pass_downsampled_->size()<<endl;
  cout << "Is dense? "<<cloud_pass_downsampled_->is_dense <<endl;
  cout << "Widht: "<<cloud_pass_downsampled_->width<<" and height: "<< cloud_pass_downsampled_->height<<endl;
  
  if(counter < 10){
	counter++;
  }else{
  	//Track the object
	//tracker_->setInputCloud (cloud_pass_downsampled_filtered);
	tracker_->setInputCloud (cloud_pass_downsampled_);
	tracker_->compute ();
	new_cloud_ = true;
  }
}

int
main (int argc, char** argv)
{
//  if (argc < 3)
//    {
//      PCL_WARN("Please set device_id pcd_filename(e.g. $ %s '#1' sample.pcd)\n", argv[0]);
//      exit (1);
//    }

  
  // Ros Node Initialization
  ros::init(argc, argv, "kinect2_tracking_node");
  ros::NodeHandle n;
  result_pub = n.advertise<geometry_msgs::Pose>("object_tracking_result", 1000);
  
  //Initialize Variables
  int MaximumParticleNum, ParticleNum, IterationNum;
  float size_x, size_y, size_z, size_roll, size_pitch, size_yaw;
  double Delta, Epsilon, LikelihoodThr;
  bool UseNormal;
  
  // Read Parameters
  /* Path For Reference PCD file*/
  if(n.getParam("tracker/pcd_path",ref_pcd_file)){cout<<"PCD PATH: \n"<< ref_pcd_file.c_str()<<endl;};
  
  /* Filter PassThrough Z Min Max */
  n.getParam("tracker/z_max", z_max);
  n.getParam("tracker/z_min", z_min);
  
  /* Grid Size For ApproximateVoxelGrid*/
  n.getParam("tracker/downsampling_grid_size", downsampling_grid_size_);
  
  /* Parameters for  KLDAdaptiveParticleFilterOMPTracker */
  n.getParam("tracker/KDLAdaptive/MaximumParticleNum",MaximumParticleNum);
  n.getParam("tracker/KDLAdaptive/Delta",Delta);
  n.getParam("tracker/KDLAdaptive/Epsilon",Epsilon);
  
  /* Parameters for  ParticleFilter */
  n.getParam("tracker/ParticleFilter/LikelihoodThr",LikelihoodThr);
  n.getParam("tracker/ParticleFilter/UseNormal",UseNormal);
  n.getParam("tracker/ParticleFilter/ParticleNum",ParticleNum);
  n.getParam("tracker/ParticleFilter/IterationNum",IterationNum);
  
  /* Bin Size */
  n.getParam("tracker/bin_size/x",size_x);
  n.getParam("tracker/bin_size/y",size_y);
  n.getParam("tracker/bin_size/z",size_z);
  n.getParam("tracker/bin_size/roll",size_roll);
  n.getParam("tracker/bin_size/pitch",size_pitch);
  n.getParam("tracker/bin_size/yaw",size_yaw);
  
  // Get From Parameter Server
  
  
  
  
  //read pcd file
  target_cloud.reset(new Cloud());
  if(pcl::io::loadPCDFile (ref_pcd_file, *target_cloud) == -1){
    std::cout << "pcd file not found" << std::endl;
    exit(-1);
  }

  //std::string device_id = std::string (argv[1]);  

  counter = 0;

  //Set parameters
  new_cloud_  = false;
  //downsampling_grid_size_ =  0.003;

  std::vector<double> default_step_covariance = std::vector<double> (6, 0.015 * 0.015);
  default_step_covariance[3] *= 40.0;
  default_step_covariance[4] *= 40.0;
  default_step_covariance[5] *= 40.0;

  std::vector<double> initial_noise_covariance = std::vector<double> (6, 0.00001);
  std::vector<double> default_initial_mean = std::vector<double> (6, 0.0);

  boost::shared_ptr<KLDAdaptiveParticleFilterOMPTracker<RefPointType, ParticleT> > tracker
    (new KLDAdaptiveParticleFilterOMPTracker<RefPointType, ParticleT> (8));

  ParticleT bin_size;
  bin_size.x = size_x;
  bin_size.y = size_y;
  bin_size.z = size_z;
  bin_size.roll = size_roll;
  bin_size.pitch = size_pitch;
  bin_size.yaw = size_yaw;


  //Set all parameters for  KLDAdaptiveParticleFilterOMPTracker
  tracker->setMaximumParticleNum (MaximumParticleNum);
  tracker->setDelta (Delta);
  tracker->setEpsilon (Epsilon);
  tracker->setBinSize (bin_size);

  //Set all parameters for  ParticleFilter
  tracker_ = tracker;
  tracker_->setTrans (Eigen::Affine3f::Identity ());
  tracker_->setStepNoiseCovariance (default_step_covariance);
  tracker_->setInitialNoiseCovariance (initial_noise_covariance);
  tracker_->setInitialNoiseMean (default_initial_mean);
  tracker_->setIterationNum (IterationNum);
  tracker_->setParticleNum (ParticleNum);
  tracker_->setResampleLikelihoodThr(LikelihoodThr);
  tracker_->setUseNormal (UseNormal);


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
  //CloudPtr transed_ref_downsampled_dense (new Cloud);
  //std::vector<int> ref_indices;
  
  
  /* Reference Point Cloud Filtering & Transformations */
  pcl::compute3DCentroid<RefPointType> (*target_cloud, c);
  trans.translation ().matrix () = Eigen::Vector3f (c[0], c[1], c[2]);
  pcl::transformPointCloud<RefPointType> (*target_cloud, *transed_ref, trans.inverse());
  gridSampleApprox (transed_ref, *transed_ref_downsampled, downsampling_grid_size_);
  //pcl::removeNaNFromPointCloud(*transed_ref_downsampled, *transed_ref_downsampled_dense, ref_indices);
  
  //set reference model and trans
  tracker_->setReferenceCloud (transed_ref_downsampled);
  tracker_->setTrans (trans);


  //Setup OpenNIGrabber and viewer
  pcl::visualization::CloudViewer* viewer_ = new pcl::visualization::CloudViewer("PCL OpenNI2 Tracking Viewer");
  pcl::Grabber* interface = new pcl::io::OpenNI2Grabber();
  boost::function<void (const CloudConstPtr&)> f =
    boost::bind (&cloud_cb, _1);
  interface->registerCallback (f);
    
  viewer_->runOnVisualizationThread (boost::bind(&viz_cb, _1), "viz_cb");

  //Start viewer and object tracking
  interface->start();
  //ros::spin();
  while (!viewer_->wasStopped ())
    boost::this_thread::sleep(boost::posix_time::seconds(1));
  interface->stop();
}
