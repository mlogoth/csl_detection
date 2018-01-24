# csl_detection
Object Detection and Tracking Algorithms based on PCL

## Software
The package is tested on Ubuntu 16.04.2 LTS with ROS Kinetic. 

## Hardware 
The __Microsoft Kinect Sensor V2__ is used  to get the point cloud data. The Microsoft Kinect Sensor V1 could also be used.
 
##Dependences

###Kinect V2 Drivers
* **Openni 2 Drivers**:

```sh 
sudo apt-get install libopenni2-0 libopenni2-dev openni2-utils openni2-doc
```
Test it by:
```sh
NiViewer2
```
 
### PCL 1.8.0
For installation instrunctions follow the link bellow:

[PCL 1.8.0 Installation Instructions](https://askubuntu.com/questions/916260/how-to-install-point-cloud-library-v1-8-pcl-1-8-0-on-ubuntu-16-04-2-lts-for/916261)

## Build
Clone the repository in your catkin workspace and build the packages:
```bash
git clone https://github.com/mlogoth/csl_detection.git
roscd && cd ..
catkin_make
```

##Run The Algorithms

Launch a roscore:
```sh
roscore
```

* **Test The Kinect2 Grabber**

The ```pcl::io::openNI2Grabber()```  is used to read the point cloud from the Kinect sensor.  Test the grabber by running:

```sh
rosrun csl_detection openni2_grabber_test 
```
* **Known Object Tracking Algorithm **
More explanations [here](http://pointclouds.org/documentation/tutorials/tracking.php).
```sh
roslaunch csl_detection kinect2_tracker.launch
```
You can change the parameters of the tracker and the path of the ```.pcd``` file that is used by editing the ```csl_detection/config/AdaptiveOMPLTracker.yaml``` file:
```yaml
tracker:
    pcd_path: /home/mike/Downloads/coke_csl.pcd
    z_max: 4.0
    z_min: 0.01
    downsampling_grid_size: 0.003
    KDLAdaptive:
        MaximumParticleNum: 1000
        Delta: 0.99
        Epsilon: 0.2
    ParticleFilter:
        LikelihoodThr: 0.00
        UseNormal: false
        ParticleNum: 600
        IterationNum: 1
    bin_size:
        x: 0.1
        y: 0.1
        z: 0.1
        roll: 0.1
        pitch: 0.1
        yaw: 0.1
```
:bangbang: ```PCD``` files are included in the ```csl_detection/data``` folder.

* **Model Free Object Detection and Tracking**
```sh
rosrun csl_detection dian_csl_tracker
```

## Useful Tools
1.  **PCD Cloud Viewer**
 ```
  rosrun csl_detection pcd_cloud_viewer 
 ```
  file:=~/Downloads/coke.pcd
  
  
2. **Mesh (obj/ply) to CPD**
  ```
   pcl_mesh2pcd input.(ply/obj) output.pcd 
   ```  
  for more info: ```pcl_mesh2pcd -h```
  Also you can use:
  ```pcl_ply2pcd input.ply output.pcd```

