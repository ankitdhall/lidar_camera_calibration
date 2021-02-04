#include <cstdlib>
#include <cstdio>
#include <iostream>
#include <map>
#include <fstream>
#include <cmath>

#include "opencv2/opencv.hpp"
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

#include <ros/package.h>

#include <pcl_ros/point_cloud.h>
#include <boost/foreach.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <velodyne_pcl/point_types.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/common/intersections.h>


#include "lidar_camera_calibration/Utils.h"

int iteration_count = 0;
std::vector <std::vector<cv::Point>> stored_corners;

bool getCorners(cv::Mat img, pcl::PointCloud <pcl::PointXYZ> scan, cv::Mat P, int num_of_markers, int MAX_ITERS) {

    ROS_INFO_STREAM("iteration number: " << iteration_count << "\n");

    // Masking happens here
    cv::Mat edge_mask = cv::Mat::zeros(img.size(), CV_8UC1);
    edge_mask(cv::Rect(0, 0, img.cols, img.rows)) = 1;
    img.copyTo(edge_mask, edge_mask);

    img = edge_mask;

    pcl::PointCloud <pcl::PointXYZ> pc = scan;
    // scan = Velodyne::Velodyne(filtered_pc);

    cv::Rect frame(0, 0, img.cols, img.rows);

    cv::Mat image_edge_laser = project(P, frame, scan, NULL);
    cv::threshold(image_edge_laser, image_edge_laser, 10, 255, 0);


    cv::Mat combined_rgb_laser;
    std::vector <cv::Mat> rgb_laser_channels;

    rgb_laser_channels.push_back(image_edge_laser);
    rgb_laser_channels.push_back(cv::Mat::zeros(image_edge_laser.size(), CV_8UC1));
    rgb_laser_channels.push_back(img);

    cv::merge(rgb_laser_channels, combined_rgb_laser);

    std::map <std::pair<int, int>, std::vector<float>> c2D_to_3D;
    std::vector<float> point_3D;

    // store correspondences
    for (pcl::PointCloud<pcl::PointXYZ>::iterator pt = pc.points.begin(); pt < pc.points.end(); pt++) {

        // ignore points behind the camera
        if (pt->z < 0) {
            continue;
        }

        cv::Point xy = project(*pt, P);
        if (xy.inside(frame)) {
            // create a map of 2D and 3D points
            point_3D.clear();
            point_3D.push_back(pt->x);
            point_3D.push_back(pt->y);
            point_3D.push_back(pt->z);
            c2D_to_3D[std::pair<int, int>(xy.x, xy.y)] = point_3D;
        }
    }

    // get region of interest
    std::vector<int> LINE_SEGMENTS(num_of_markers, 4);  // assuming each has 4 edges and 4 corners

    pcl::PointCloud<pcl::PointXYZ>::Ptr board_corners(new pcl::PointCloud <pcl::PointXYZ>);

    pcl::PointCloud<pcl::PointXYZ>::Ptr marker(new pcl::PointCloud <pcl::PointXYZ>);
    std::vector <cv::Point3f> c_3D;
    std::vector <cv::Point2f> c_2D;


    cv::namedWindow("cloud", cv::WINDOW_NORMAL);
    cv::namedWindow("polygon", cv::WINDOW_NORMAL);

    std::string pkg_loc = ros::package::getPath("lidar_camera_calibration");
    std::ofstream outfile(pkg_loc + "/conf/points.txt", std::ios_base::trunc);
    outfile << num_of_markers * 4 << "\n";

    for (int q = 0; q < num_of_markers; q++) {
        std::cout << "---------Moving on to next marker--------\n";
        std::vector <Eigen::VectorXf> line_model;
        for (int i = 0; i < LINE_SEGMENTS[q]; i++) {
            cv::Point _point_;
            std::vector <cv::Point> polygon;
            int collected;

            // get markings in the first iteration only
            if (iteration_count == 0) {
                polygon.clear();
                collected = 0;
                while (collected != LINE_SEGMENTS[q]) {

                    cv::setMouseCallback("cloud", onMouse, &_point_);

                    cv::imshow("cloud", image_edge_laser);
                    cv::waitKey(0);
                    ++collected;
                    polygon.push_back(_point_);
                }
                stored_corners.push_back(polygon);
            }

            polygon = stored_corners[4 * q + i];

            cv::Mat polygon_image = cv::Mat::zeros(image_edge_laser.size(), CV_8UC1);

            rgb_laser_channels.clear();
            rgb_laser_channels.push_back(image_edge_laser);
            rgb_laser_channels.push_back(cv::Mat::zeros(image_edge_laser.size(), CV_8UC1));
            rgb_laser_channels.push_back(cv::Mat::zeros(image_edge_laser.size(), CV_8UC1));
            cv::merge(rgb_laser_channels, combined_rgb_laser);

            for (int j = 0; j < 4; j++) {
                cv::line(combined_rgb_laser, polygon[j],
                         polygon[(j + 1) % 4],
                         cv::Scalar(0, 255, 0));
            }

            // initialize PointClouds
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud <pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr final(new pcl::PointCloud <pcl::PointXYZ>);

            for (std::map < std::pair < int, int >, std::vector < float > > ::iterator it = c2D_to_3D.begin();
                    it != c2D_to_3D.end();
            ++it)
            {

                if (cv::pointPolygonTest(cv::Mat(polygon), cv::Point(it->first.first, it->first.second), true) > 0) {
                    cloud->push_back(pcl::PointXYZ(it->second[0], it->second[1], it->second[2]));
                    rectangle(combined_rgb_laser,
                              cv::Point(it->first.first, it->first.second),
                              cv::Point(it->first.first, it->first.second),
                              cv::Scalar(0, 0, 255), 3, 8,
                              0); // RED point
                }
            }

            if (cloud->size() < 2) { return false; }

            cv::imshow("polygon", combined_rgb_laser);
            cv::waitKey(4);

            std::vector<int> inliers;
            Eigen::VectorXf model_coefficients;


            // created RandomSampleConsensus object and compute the appropriated model
            pcl::SampleConsensusModelLine<pcl::PointXYZ>::Ptr model_l(
                    new pcl::SampleConsensusModelLine<pcl::PointXYZ>(cloud));

            pcl::RandomSampleConsensus <pcl::PointXYZ> ransac(model_l);
            ransac.setDistanceThreshold(0.01);
            ransac.computeModel();
            ransac.getInliers(inliers);
            ransac.getModelCoefficients(model_coefficients);
            line_model.push_back(model_coefficients);

            std::cout << "Line coefficients are:" << "\n" << model_coefficients << "\n";
            // copies all inliers of the model computed to another PointCloud
            pcl::copyPointCloud<pcl::PointXYZ>(*cloud, inliers, *final);
            *marker += *final;
        }



        // calculate approximate intersection of lines
        Eigen::Vector4f p1, p2, p_intersect;
        pcl::PointCloud<pcl::PointXYZ>::Ptr corners(new pcl::PointCloud <pcl::PointXYZ>);
        for (int i = 0; i < LINE_SEGMENTS[q]; i++) {
            pcl::lineToLineSegment(line_model[i], line_model[(i + 1) % LINE_SEGMENTS[q]], p1, p2);
            for (int j = 0; j < 4; j++) {
                p_intersect(j) = (p1(j) + p2(j)) / 2.0;
            }
            c_3D.push_back(cv::Point3f(p_intersect(0), p_intersect(1), p_intersect(2)));
            corners->push_back(pcl::PointXYZ(p_intersect(0), p_intersect(1), p_intersect(2)));
            std::cout << "Point of intersection is approximately: \n" << p_intersect << "\n";
            std::cout << p_intersect(0) << " " << p_intersect(1) << " " << p_intersect(2) << "\n";
            outfile << p_intersect(0) << " " << p_intersect(1) << " " << p_intersect(2) << "\n";

        }

        *board_corners += *corners;

        std::cout << "Distance between the corners:\n";
        for (int i = 0; i < 4; i++) {
            std::cout <<
                      sqrt(
                              pow(c_3D[4 * q + i].x - c_3D[4 * q + (i + 1) % 4].x, 2)
                              + pow(c_3D[4 * q + i].y - c_3D[4 * q + (i + 1) % 4].y, 2)
                              + pow(c_3D[4 * q + i].z - c_3D[4 * q + (i + 1) % 4].z, 2)
                      )
                      << std::endl;
        }


    }
    outfile.close();

    iteration_count++;
    if (iteration_count == MAX_ITERS) {
        ros::shutdown();
    }
    return true;
}
