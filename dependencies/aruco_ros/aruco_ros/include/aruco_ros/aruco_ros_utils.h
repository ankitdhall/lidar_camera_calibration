#ifndef ARUCO_ROS_UTILS_H
#define ARUCO_ROS_UTILS_H

#include <aruco/aruco.h>
#include <sensor_msgs/CameraInfo.h>
#include <tf/transform_datatypes.h>

namespace aruco_ros
{
  /**
     * @brief rosCameraInfo2ArucoCamParams gets the camera intrinsics from a CameraInfo message and copies them
     *                                     to aruco_ros own data structure
     * @param cam_info
     * @param useRectifiedParameters if true, the intrinsics are taken from cam_info.P and the distortion parameters
     *                               are set to 0. Otherwise, cam_info.K and cam_info.D are taken.
     * @return
     */
  aruco::CameraParameters rosCameraInfo2ArucoCamParams(const sensor_msgs::CameraInfo& cam_info,
                                                       bool useRectifiedParameters);

  //FIXME: make parameter const as soon as the used function is also const
  tf::Transform arucoMarker2Tf(const aruco::Marker& marker, bool rotate_marker_axis=true);

}
#endif // ARUCO_ROS_UTILS_H
