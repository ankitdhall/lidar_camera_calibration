#include <aruco_ros/aruco_ros_utils.h>
#include <ros/console.h>
#include <ros/assert.h>
#include <iostream>
#include <tf/transform_datatypes.h>

aruco::CameraParameters aruco_ros::rosCameraInfo2ArucoCamParams(const sensor_msgs::CameraInfo& cam_info,
                                                                bool useRectifiedParameters)
{
    cv::Mat cameraMatrix(3, 3, CV_64FC1);
    cv::Mat distorsionCoeff(4, 1, CV_64FC1);
    cv::Size size(cam_info.height, cam_info.width);

    if ( useRectifiedParameters )
    {
      cameraMatrix.setTo(0);
      cameraMatrix.at<double>(0,0) = cam_info.P[0];   cameraMatrix.at<double>(0,1) = cam_info.P[1];   cameraMatrix.at<double>(0,2) = cam_info.P[2];
      cameraMatrix.at<double>(1,0) = cam_info.P[4];   cameraMatrix.at<double>(1,1) = cam_info.P[5];   cameraMatrix.at<double>(1,2) = cam_info.P[6];
      cameraMatrix.at<double>(2,0) = cam_info.P[8];   cameraMatrix.at<double>(2,1) = cam_info.P[9];   cameraMatrix.at<double>(2,2) = cam_info.P[10];

      for(int i=0; i<4; ++i)
        distorsionCoeff.at<double>(i, 0) = 0;
    }
    else
    {
      for(int i=0; i<9; ++i)
        cameraMatrix.at<double>(i%3, i-(i%3)*3) = cam_info.K[i];

      if(cam_info.D.size() == 4)
      {
        for(int i=0; i<4; ++i)
          distorsionCoeff.at<double>(i, 0) = cam_info.D[i];
      }
      else
      {
        ROS_WARN("length of camera_info D vector is not 4, assuming zero distortion...");
        for(int i=0; i<4; ++i)
          distorsionCoeff.at<double>(i, 0) = 0;
      }
    }

    return aruco::CameraParameters(cameraMatrix, distorsionCoeff, size);
}

tf::Transform aruco_ros::arucoMarker2Tf(const aruco::Marker &marker, bool rotate_marker_axis)
{
    cv::Mat rot(3, 3, CV_64FC1);
    cv::Mat Rvec64;
    marker.Rvec.convertTo(Rvec64, CV_64FC1);
    cv::Rodrigues(Rvec64, rot);
    cv::Mat tran64;
    marker.Tvec.convertTo(tran64, CV_64FC1);

    // Rotate axis direction as to fit ROS (?)
    if (rotate_marker_axis)
    {
      cv::Mat rotate_to_ros(3, 3, CV_64FC1);
      // -1 0 0
      // 0 0 1
      // 0 1 0
      rotate_to_ros.at<double>(0,0) = -1.0;
      rotate_to_ros.at<double>(0,1) = 0.0;
      rotate_to_ros.at<double>(0,2) = 0.0;
      rotate_to_ros.at<double>(1,0) = 0.0;
      rotate_to_ros.at<double>(1,1) = 0.0;
      rotate_to_ros.at<double>(1,2) = 1.0;
      rotate_to_ros.at<double>(2,0) = 0.0;
      rotate_to_ros.at<double>(2,1) = 1.0;
      rotate_to_ros.at<double>(2,2) = 0.0;
      rot = rot*rotate_to_ros.t();
    }
    tf::Matrix3x3 tf_rot(rot.at<double>(0,0), rot.at<double>(0,1), rot.at<double>(0,2),
                         rot.at<double>(1,0), rot.at<double>(1,1), rot.at<double>(1,2),
                         rot.at<double>(2,0), rot.at<double>(2,1), rot.at<double>(2,2));

    tf::Vector3 tf_orig(tran64.at<double>(0,0), tran64.at<double>(1,0), tran64.at<double>(2,0));


    return tf::Transform(tf_rot, tf_orig);
}
