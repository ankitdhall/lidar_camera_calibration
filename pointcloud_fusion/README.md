# Fusing 2 point clouds

Provides a C++ script to fuse two point clouds from two stereo cameras.  

## Prerequisites:

* [PCL](http://pointclouds.org/)  
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page)  

## Getting started
Place point cloud files `pointcloud_fusion/data/pcd_pos1/pcd_2.pcd` and `pointcloud_fusion/data/pcd_pos2/pcd_2.pcd`.  

To fuse the two point clouds, one needs the transformation estimated by the `lidar_camera_calibration find_transformation.launch` node, which is a rotation and translation as a 4x4 matrix that transform all points in the LiDAR frame to the camera frame.  

In `pointcloud_fusion/data/rts.txt` provide,  
T1, transformation from LiDAR-to-Camera1 (corresponding `.pcd` file in `pcd_pos1`)
T2, transformation from LiDAR-to-Camera2 (corresponding `.pcd` file in `pcd_pos2`)  

## Usage
Go to `pointcloud_fusion/build` and compile the program.

```shell
cmake ..
make
```

If you encounted an error, something that looks like this,  
>CMake Error: The current CMakeCache.txt directory /home/user-name/lidar_camera_calibration/pointcloud_fusion/build/CMakeCache.txt is different than the directory /home/user-name-2/lidar_camera_calibration/pointcloud_fusion/build/CMakeCache.txt where CMakeCache.txt was created. This may result in binaries being created in the wrong place. If you are not sure, reedit the CMakeCache.txt

then, delete the CMakeCache.txt and compile the program.

Run  
```shell
./fusion
```
to fuse the point clouds.  

The fused point cloud will be saved in `pointcloud_fusion/data` as `final_cloud.pcd` and can be viewed using `pcl_viewer`.