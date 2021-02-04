#include <cstdlib>
#include <cstdio>
#include <iostream>
#include <map>
#include <fstream>
#include <cmath>

#include "opencv2/opencv.hpp"
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

#include <pcl_ros/point_cloud.h>
#include <boost/foreach.hpp>
#include <pcl_conversions/pcl_conversions.h>

#include <linux/version.h>

#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 4, 0)
#include <velodyne_pointcloud/point_types.h>
#else

#include <velodyne_pcl/point_types.h>

#endif

#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/common/intersections.h>

cv::Point project(const pcl::PointXYZ &pt, const cv::Mat &projection_matrix) {
    cv::Mat pt_3D(4, 1, CV_32FC1);

    pt_3D.at<float>(0) = pt.x;
    pt_3D.at<float>(1) = pt.y;
    pt_3D.at<float>(2) = pt.z;
    pt_3D.at<float>(3) = 1.0f;

    cv::Mat pt_2D = projection_matrix * pt_3D;

    float w = pt_2D.at<float>(2);
    float x = pt_2D.at<float>(0) / w;
    float y = pt_2D.at<float>(1) / w;
    return cv::Point(x, y);
}

cv::Mat project(cv::Mat projection_matrix, cv::Rect frame, pcl::PointCloud <pcl::PointXYZ> point_cloud,
                pcl::PointCloud <pcl::PointXYZ> *visible_points) {
    cv::Mat plane = cv::Mat::zeros(frame.size(), CV_32FC1);

    for (pcl::PointCloud<pcl::PointXYZ>::iterator pt = point_cloud.points.begin();
         pt < point_cloud.points.end(); pt++) {

        // behind the camera
        if (pt->z < 0) {
            continue;
        }

        //float intensity = pt->intensity;
        cv::Point xy = project(*pt, projection_matrix);
        if (xy.inside(frame)) {
            if (visible_points != NULL) {
                visible_points->push_back(*pt);
            }

            plane.at<float>(xy) = 250;
        }
    }

    cv::Mat plane_gray;
    cv::normalize(plane, plane_gray, 0, 255, cv::NORM_MINMAX, CV_8UC1);
    cv::dilate(plane_gray, plane_gray, cv::Mat());

    return plane_gray;
}

void onMouse(int event, int x, int y, int f, void *g) {

    cv::Point *P = static_cast<cv::Point *>(g);
    switch (event) {

        case cv::EVENT_LBUTTONDOWN :
            P->x = x;
            P->y = y;
            break;

        case cv::EVENT_LBUTTONUP   :
            P->x = x;
            P->y = y;
            break;

        default                     :
            break;


    }

}
