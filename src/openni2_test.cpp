#include <iostream>

#include <mutex>
#include <pcl/io/openni2_grabber.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>

using namespace std ;
typedef pcl::PointXYZRGBA T;
typedef pcl::PointCloud<T> CloudType;
typedef CloudType::Ptr CloudPtr;
typedef CloudType::ConstPtr CloudConstPtr;




class SimpleOpenNIViewer {
public:
  SimpleOpenNIViewer()
      : viewer(new pcl::visualization::PCLVisualizer("PCL OpenNI Viewer")) {}

 //Filter along a specified dimension
void filterPassThrough (const CloudConstPtr &cloud, CloudType &result, bool keep_organized)
{
  pcl::PassThrough<T> pass;
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.05, 10.0);
  pass.setKeepOrganized (keep_organized);
  pass.setInputCloud (cloud);
  pass.filter (result);
}

void gridSampleApprox (const CloudConstPtr &cloud, CloudType &result, double leaf_size)
{
  pcl::ApproximateVoxelGrid<T> grid;
  grid.setLeafSize (static_cast<float> (leaf_size), static_cast<float> (leaf_size), static_cast<float> (leaf_size));
  grid.setInputCloud (cloud);
  grid.filter (result);
}


  void cloud_cb_(const pcl::PointCloud<T>::ConstPtr &cloud) {
    if (!viewer->wasStopped()) {
      mtx.lock();
      CloudPtr outputCloudfiltered3 (new CloudType);
      
      cout<<"**********************************************"<<endl;
      cout<<"Size: "<<cloud->size()<<endl;
      cout<<"Is Dense? "<<cloud->is_dense<<endl;
      cout<<"The point Cloud's Widht: "<<cloud->width<<" and height: "<< cloud->height<<endl;
      cout<<"\n";
      
      cout<<"--- Filter Pass Through Organized ---"<<endl;
      CloudType::Ptr outputCloud (new CloudType);
      SimpleOpenNIViewer::filterPassThrough(cloud, *outputCloud, true);
      //pcl::removeNaNFromPointCloud(*cloud, *outputCloud, indices);
      cout<<"Size: "<<outputCloud->size()<<endl;
      cout<<"Is Dense? "<<outputCloud->is_dense<<endl;
      cout<<"The point Cloud's Widht: "<<outputCloud->width<<" and height: "<< outputCloud->height<<endl;
      
      
      cout<<"\n";
      cout<<"--- Filter Pass Through Not Organized ---"<<endl;
      CloudPtr outputCloudfiltered (new CloudType);
      SimpleOpenNIViewer::filterPassThrough(cloud, *outputCloudfiltered, false);
      cout<<"Size: "<<outputCloudfiltered->size()<<endl;
      cout<<"Is Dense? "<<outputCloudfiltered->is_dense<<endl;
      cout<<"The point Cloud's Widht: "<<outputCloudfiltered->width<<" and height: "<< outputCloudfiltered->height<<endl;
      
      cout<<"\n";
      cout<<"-- After Removing  NaN  Of Not Organized --"<<endl;
      CloudPtr outputCloudfiltered2 (new CloudType);
      std::vector<int> indices;
      pcl::removeNaNFromPointCloud(*outputCloudfiltered, *outputCloudfiltered2, indices);
      cout<<"Size: "<<outputCloudfiltered2->size()<<endl;
      cout<<"Is Dense? "<<outputCloudfiltered2->is_dense<<endl;
      cout<<"The point Cloud's Widht: "<<outputCloudfiltered2->width<<" and height: "<< outputCloudfiltered2->height<<endl;
      
      cout<<"\n";
      cout<<"-- Approximate Voxel Grid Filter --"<<endl;
      SimpleOpenNIViewer::gridSampleApprox(outputCloudfiltered2, *outputCloudfiltered3, 0.001);
      cout<<"Size: "<<outputCloudfiltered3->size()<<endl;
      cout<<"Is Dense? "<<outputCloudfiltered3->is_dense<<endl;
      cout<<"The point Cloud's Widht: "<<outputCloudfiltered3->width<<" and height: "<< outputCloudfiltered3->height<<endl;
      
      // rgba color Handler
      pcl::visualization::PointCloudColorHandlerRGBAField<pcl::PointXYZRGBA> rgb(outputCloudfiltered3);
      // One Color Handler
      pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> single_color(outputCloudfiltered3, 0, 255, 0);
      viewer->updatePointCloud<T>(outputCloudfiltered3, single_color,"openni");
      mtx.unlock();
    }
  }

  void run() {

   // OpenNI2 Grabber Parameters
   std::string device_id ("");
   pcl::io::OpenNI2Grabber::Mode depth_mode = pcl::io::OpenNI2Grabber::OpenNI_Default_Mode;
   pcl::io::OpenNI2Grabber::Mode image_mode = pcl::io::OpenNI2Grabber::OpenNI_Default_Mode;
   
//    pcl::Grabber *interface = new pcl::io::OpenNI2Grabber(
//        "", pcl::io::OpenNI2Grabber::OpenNI_QVGA_30Hz,
//        pcl::io::OpenNI2Grabber::OpenNI_QVGA_30Hz);
    pcl::Grabber *interface = new pcl::io::OpenNI2Grabber(device_id,depth_mode ,image_mode);


    pcl::PointCloud<T>::Ptr cloud(new pcl::PointCloud<T>);
    
    /* Viewer Initialization */
    pcl::visualization::PointCloudColorHandlerRGBAField<pcl::PointXYZRGBA> rgb(cloud);
    viewer->addPointCloud<T>(cloud, rgb,"openni");
    viewer->addCoordinateSystem (1.0);
    viewer->setBackgroundColor(0,0,0);
    boost::function<void(const pcl::PointCloud<T>::ConstPtr &)> f =
        boost::bind(&SimpleOpenNIViewer::cloud_cb_, this, _1);

    interface->registerCallback(f);
    interface->start();

    while (!viewer->wasStopped()) {
      boost::this_thread::sleep(boost::posix_time::millisec(50));
      mtx.lock();
      viewer->spinOnce();
      mtx.unlock();
    }

    interface->stop();
  }

  std::mutex mtx;
  pcl::visualization::PCLVisualizer::Ptr viewer;
};

int main() {
  SimpleOpenNIViewer v;
  v.run();
  return 0;
}

