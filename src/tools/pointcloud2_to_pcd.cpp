#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/PointCloud2.h"

#include <sstream>
#include <cstdio>
#include <cmath>

#include <pcl/common/centroid.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>



using namespace std;
pcl::PointCloud<pcl::PointXYZRGBA> cloud;



void getCloud(const sensor_msgs::PointCloud2& msg)
{
    static int count_ = 0;
    pcl::fromROSMsg(msg, cloud);
    cout<<">> Read Object Point Cloud..."<<endl;
    cout<<"* Object Cloud Size: "<<cloud.points.size()<<endl;
    
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr transed_ref (new pcl::PointCloud<pcl::PointXYZRGBA>);
    Eigen::Vector4f c;
    
    pcl::compute3DCentroid<pcl::PointXYZRGBA> (cloud, c);
    
    Eigen::Affine3f trans = Eigen::Affine3f::Identity ();
    trans.translation ().matrix () = Eigen::Vector3f (c[0], c[1], c[2]);
    
    pcl::transformPointCloud<pcl::PointXYZRGBA> (cloud, *transed_ref, trans.inverse());
    
    if (count_ == 10)
    {
        pcl::io::savePCDFile( "cloud_detection.pcd", *transed_ref, true); // Binary format
        cout<< "PCD File written!"<<endl;
        
    }
    count_++;

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pointcloud2pcd");
    ros::NodeHandle n;
    ros::Rate loop_rate(10);
    
    ros::Subscriber sub = n.subscribe("/object_detected_cloud", 100, getCloud);
    while (ros::ok())
    {
        ros::spinOnce();
    }
    
  return 0;
}
