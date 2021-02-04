#include <cstdlib>
#include <cstdio>
#include <math.h>
#include <algorithm>
#include <map>

#include "opencv2/opencv.hpp"

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <camera_info_manager/camera_info_manager.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <pcl_ros/point_cloud.h>
#include <boost/foreach.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <velodyne_pcl/point_types.h>
//#include <velodyne_pointcloud/point_types.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>

#include "lidar_camera_calibration/Corners.h"
#include "lidar_camera_calibration/PreprocessUtils.h"
#include "lidar_camera_calibration/Find_RT.h"

#include "lidar_camera_calibration/marker_6dof.h"

using namespace cv;
using namespace std;
using namespace ros;
using namespace message_filters;
using namespace pcl;


string CAMERA_INFO_TOPIC;
string VELODYNE_TOPIC;


Mat projection_matrix;

pcl::PointCloud <myPointXYZRID> point_cloud;
Hesai::PointCloud point_cloud_hesai;

Eigen::Quaterniond qlidarToCamera;
Eigen::Matrix3d lidarToCamera;


void callback_noCam(const sensor_msgs::PointCloud2ConstPtr &msg_pc,
                    const lidar_camera_calibration::marker_6dof::ConstPtr &msg_rt) {
    ROS_INFO_STREAM("Velodyne scan received at " << msg_pc->header.stamp.toSec());
    ROS_INFO_STREAM("marker_6dof received at " << msg_rt->header.stamp.toSec());

    // Loading Velodyne point cloud_sub
    if (config.lidar_type == 0) // velodyne lidar
    {
        fromROSMsg(*msg_pc, point_cloud);
    } else if (config.lidar_type == 1) //hesai lidar
    {
        fromROSMsg(*msg_pc, point_cloud_hesai);
        point_cloud = *(toMyPointXYZRID(point_cloud_hesai));
    }

    point_cloud = transform(point_cloud, config.initialTra[0], config.initialTra[1], config.initialTra[2],
                            config.initialRot[0], config.initialRot[1], config.initialRot[2]);

    //Rotation matrix to transform lidar point cloud to camera's frame

    qlidarToCamera = Eigen::AngleAxisd(config.initialRot[2], Eigen::Vector3d::UnitZ())
                     * Eigen::AngleAxisd(config.initialRot[1], Eigen::Vector3d::UnitY())
                     * Eigen::AngleAxisd(config.initialRot[0], Eigen::Vector3d::UnitX());

    lidarToCamera = qlidarToCamera.matrix();

    std::cout << "\n\nInitial Rot" << lidarToCamera << "\n";
    point_cloud = intensityByRangeDiff(point_cloud, config);
    // x := x, y := -z, z := y

    cv::Mat temp_mat(config.s, CV_8UC3);
    pcl::PointCloud <pcl::PointXYZ> retval = *(toPointsXYZ(point_cloud));

    std::vector<float> marker_info;

    for (std::vector<float>::const_iterator it = msg_rt->dof.data.begin(); it != msg_rt->dof.data.end(); ++it) {
        marker_info.push_back(*it);
        std::cout << *it << " ";
    }
    std::cout << "\n";

    bool no_error = getCorners(temp_mat, retval, config.P, config.num_of_markers, config.MAX_ITERS);
    if (no_error) {
        find_transformation(marker_info, config.num_of_markers, config.MAX_ITERS, lidarToCamera);
    }
}

void callback(const sensor_msgs::CameraInfoConstPtr &msg_info,
              const sensor_msgs::PointCloud2ConstPtr &msg_pc,
              const lidar_camera_calibration::marker_6dof::ConstPtr &msg_rt) {

    ROS_INFO_STREAM("Camera info received at " << msg_info->header.stamp.toSec());
    ROS_INFO_STREAM("Velodyne scan received at " << msg_pc->header.stamp.toSec());
    ROS_INFO_STREAM("marker_6dof received at " << msg_rt->header.stamp.toSec());

    float p[12];
    float *pp = p;
    for (boost::array<double, 12ul>::const_iterator i = msg_info->P.begin(); i != msg_info->P.end(); i++) {
        *pp = (float) (*i);
        pp++;
    }
    cv::Mat(3, 4, CV_32FC1, &p).copyTo(projection_matrix);

    // Loading Velodyne point cloud_sub
    if (config.lidar_type == 0) // velodyne lidar
    {
        fromROSMsg(*msg_pc, point_cloud);
    } else if (config.lidar_type == 1) // hesai lidar
    {
        fromROSMsg(*msg_pc, point_cloud_hesai);
        point_cloud = *(toMyPointXYZRID(point_cloud_hesai));
    }

    point_cloud = transform(point_cloud, config.initialTra[0], config.initialTra[1], config.initialTra[2],
                            config.initialRot[0], config.initialRot[1], config.initialRot[2]);

    //Rotation matrix to transform lidar point cloud to camera's frame

    qlidarToCamera = Eigen::AngleAxisd(config.initialRot[2], Eigen::Vector3d::UnitZ())
                     * Eigen::AngleAxisd(config.initialRot[1], Eigen::Vector3d::UnitY())
                     * Eigen::AngleAxisd(config.initialRot[0], Eigen::Vector3d::UnitX());

    lidarToCamera = qlidarToCamera.matrix();

    point_cloud = intensityByRangeDiff(point_cloud, config);
    // x := x, y := -z, z := y

    cv::Mat temp_mat(config.s, CV_8UC3);
    pcl::PointCloud <pcl::PointXYZ> retval = *(toPointsXYZ(point_cloud));

    std::vector<float> marker_info;

    for (std::vector<float>::const_iterator it = msg_rt->dof.data.begin(); it != msg_rt->dof.data.end(); ++it) {
        marker_info.push_back(*it);
        std::cout << *it << " ";
    }
    std::cout << "\n";

    getCorners(temp_mat, retval, projection_matrix, config.num_of_markers, config.MAX_ITERS);
    find_transformation(marker_info, config.num_of_markers, config.MAX_ITERS, lidarToCamera);
}


int main(int argc, char **argv) {
    readConfig();
    ros::init(argc, argv, "find_transform");

    ros::NodeHandle n;

    if (config.useCameraInfo) {
        ROS_INFO_STREAM("Reading CameraInfo from topic");
        n.getParam("/lidar_camera_calibration/camera_info_topic", CAMERA_INFO_TOPIC);
        n.getParam("/lidar_camera_calibration/velodyne_topic", VELODYNE_TOPIC);

        message_filters::Subscriber <sensor_msgs::CameraInfo> info_sub(n, CAMERA_INFO_TOPIC, 1);
        message_filters::Subscriber <sensor_msgs::PointCloud2> cloud_sub(n, VELODYNE_TOPIC, 1);
        message_filters::Subscriber <lidar_camera_calibration::marker_6dof> rt_sub(n, "lidar_camera_calibration_rt", 1);

        std::cout << "done1\n";

        typedef sync_policies::ApproximateTime <sensor_msgs::CameraInfo, sensor_msgs::PointCloud2, lidar_camera_calibration::marker_6dof> MySyncPolicy;
        Synchronizer <MySyncPolicy> sync(MySyncPolicy(10), info_sub, cloud_sub, rt_sub);
        sync.registerCallback(boost::bind(&callback, _1, _2, _3));

        ros::spin();
    } else {
        ROS_INFO_STREAM("Reading CameraInfo from configuration file");
        n.getParam("/lidar_camera_calibration/velodyne_topic", VELODYNE_TOPIC);

        message_filters::Subscriber <sensor_msgs::PointCloud2> cloud_sub(n, VELODYNE_TOPIC, 1);
        message_filters::Subscriber <lidar_camera_calibration::marker_6dof> rt_sub(n, "lidar_camera_calibration_rt", 1);

        typedef sync_policies::ApproximateTime <sensor_msgs::PointCloud2, lidar_camera_calibration::marker_6dof> MySyncPolicy;
        Synchronizer <MySyncPolicy> sync(MySyncPolicy(10), cloud_sub, rt_sub);
        sync.registerCallback(boost::bind(&callback_noCam, _1, _2));

        ros::spin();
    }

    return EXIT_SUCCESS;
}
