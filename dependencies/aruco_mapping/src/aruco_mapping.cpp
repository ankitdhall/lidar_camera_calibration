/*********************************************************************************************//**
* @file aruco_mapping.cpp
*
* Copyright (c)
* Smart Robotic Systems
* March 2015
*
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* Redistributions of source code must retain the above copyright notice, this
* list of conditions and the following disclaimer.
*
* Redistributions in binary form must reproduce the above copyright notice,
* this list of conditions and the following disclaimer in the documentation
* and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

/* Author: Jan Bacik */

#ifndef ARUCO_MAPPING_CPP
#define ARUCO_MAPPING_CPP

#include <aruco_mapping.h>
#include "lidar_camera_calibration/marker_6dof.h"

namespace aruco_mapping
{

ArucoMapping::ArucoMapping(ros::NodeHandle *nh) :
  listener_ (new tf::TransformListener),  // Initialize TF Listener  
  num_of_markers_ (10),                   // Number of used markers
  marker_size_(0.1),                      // Marker size in m
  calib_filename_("empty"),               // Calibration filepath
  space_type_ ("plane"),                  // Space type - 2D plane 
  roi_allowed_ (false),                   // ROI not allowed by default
  first_marker_detected_(false),          // First marker not detected by defualt
  lowest_marker_id_(-1),                  // Lowest marker ID
  marker_counter_(0),                     // Reset marker counter
  closest_camera_index_(0)                // Reset closest camera index 
  
{
  double temp_marker_size;  
  
  //Parse params from launch file 
  nh->getParam("/aruco_mapping/calibration_file", calib_filename_);
  nh->getParam("/aruco_mapping/marker_size", temp_marker_size); 
  nh->getParam("/aruco_mapping/num_of_markers", num_of_markers_);
  nh->getParam("/aruco_maping/space_type",space_type_);
  nh->getParam("/aruco_mapping/roi_allowed",roi_allowed_);
  nh->getParam("/aruco_mapping/roi_x",roi_x_);
  nh->getParam("/aruco_mapping/roi_y",roi_y_);
  nh->getParam("/aruco_mapping/roi_w",roi_w_);
  nh->getParam("/aruco_mapping/roi_h",roi_h_);
     
  // Double to float conversion
  marker_size_ = float(temp_marker_size);
  
  if(calib_filename_ == "empty")
    ROS_WARN("Calibration filename empty! Check the launch file paths");
  else
  {
    ROS_INFO_STREAM("Calibration file path: " << calib_filename_ );
    ROS_INFO_STREAM("Number of markers: " << num_of_markers_);
    ROS_INFO_STREAM("Marker Size: " << marker_size_);
    ROS_INFO_STREAM("Type of space: " << space_type_);
    ROS_INFO_STREAM("ROI allowed: " << roi_allowed_);
    ROS_INFO_STREAM("ROI x-coor: " << roi_x_);
    ROS_INFO_STREAM("ROI y-coor: " << roi_x_);
    ROS_INFO_STREAM("ROI width: "  << roi_w_);
    ROS_INFO_STREAM("ROI height: " << roi_h_);      
  }
    
  //ROS publishers
  marker_msg_pub_           = nh->advertise<aruco_mapping::ArucoMarker>("aruco_poses",1);
  marker_visualization_pub_ = nh->advertise<visualization_msgs::Marker>("aruco_markers",1);
  lidar_camera_calibration_rt = nh->advertise< lidar_camera_calibration::marker_6dof >("lidar_camera_calibration_rt",1);

  //Parse data from calibration file
  parseCalibrationFile(calib_filename_);

  //Initialize OpenCV window
  cv::namedWindow("Mono8", CV_WINDOW_AUTOSIZE);       
      
  //Resize marker container
  markers_.resize(num_of_markers_);
  
  // Default markers_ initialization
  for(size_t i = 0; i < num_of_markers_;i++)
  {
    markers_[i].previous_marker_id = -1;
    markers_[i].visible = false;
    markers_[i].marker_id = -1;
  }
}

ArucoMapping::~ArucoMapping()
{
 delete listener_;
}

bool
ArucoMapping::parseCalibrationFile(std::string calib_filename)
{
  sensor_msgs::CameraInfo camera_calibration_data;
  std::string camera_name = "camera";

  camera_calibration_parsers::readCalibrationIni(calib_filename, camera_name, camera_calibration_data);

  // Alocation of memory for calibration data
  cv::Mat  *intrinsics       = new(cv::Mat)(3, 3, CV_64F);
  cv::Mat  *distortion_coeff = new(cv::Mat)(5, 1, CV_64F);
  cv::Size *image_size       = new(cv::Size);

  image_size->width = camera_calibration_data.width;
  image_size->height = camera_calibration_data.height;

  for(size_t i = 0; i < 3; i++)
    for(size_t j = 0; j < 3; j++)
    intrinsics->at<double>(i,j) = camera_calibration_data.K.at(3*i+j);

  for(size_t i = 0; i < 5; i++)
    distortion_coeff->at<double>(i,0) = camera_calibration_data.D.at(i);

  ROS_DEBUG_STREAM("Image width: " << image_size->width);
  ROS_DEBUG_STREAM("Image height: " << image_size->height);
  ROS_DEBUG_STREAM("Intrinsics:" << std::endl << *intrinsics);
  ROS_DEBUG_STREAM("Distortion: " << *distortion_coeff);


  //Load parameters to aruco_calib_param_ for aruco detection
  aruco_calib_params_.setParams(*intrinsics, *distortion_coeff, *image_size);

  //Simple check if calibration data meets expected values
  if ((intrinsics->at<double>(2,2) == 1) && (distortion_coeff->at<double>(0,4) == 0))
  {
    ROS_INFO_STREAM("Calibration data loaded successfully");
    return true;
  }
  else
  {
    ROS_WARN("Wrong calibration data, check calibration file and filepath");
    return false;
  }
}

void
ArucoMapping::imageCallback(const sensor_msgs::ImageConstPtr &original_image)
{
  //Create cv_brigde instance
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr=cv_bridge::toCvCopy(original_image, sensor_msgs::image_encodings::MONO8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Not able to convert sensor_msgs::Image to OpenCV::Mat format %s", e.what());
    return;
  }
  
  // sensor_msgs::Image to OpenCV Mat structure
  cv::Mat I = cv_ptr->image;
  
  // region of interest
  if(roi_allowed_==true)
    I = cv_ptr->image(cv::Rect(roi_x_,roi_y_,roi_w_,roi_h_));

  //Marker detection
  processImage(I,I);
  
  // Show image
  cv::imshow("Mono8", I);
  cv::waitKey(10);
  /*cv::waitKey(2000);
  ros::shutdown();*/
}


bool
ArucoMapping::processImage(cv::Mat input_image,cv::Mat output_image)
{
  aruco::MarkerDetector Detector;
  std::vector<aruco::Marker> temp_markers;

  //Set visibility flag to false for all markers
  for(size_t i = 0; i < num_of_markers_; i++)
      markers_[i].visible = false;

  // Save previous marker count
  marker_counter_previous_ = marker_counter_;

  // Detect markers
  Detector.detect(input_image,temp_markers,aruco_calib_params_,marker_size_);
    
  // If no marker found, print statement
  if(temp_markers.size() == 0)
    ROS_DEBUG("No marker found!");

  //------------------------------------------------------
  // FIRST MARKER DETECTED
  //------------------------------------------------------
  if((temp_markers.size() > 0) && (first_marker_detected_ == false))
  {
    //Set flag
    first_marker_detected_=true;

    // Detect lowest marker ID
    lowest_marker_id_ = temp_markers[0].id;
    for(size_t i = 0; i < temp_markers.size();i++)
    {
      if(temp_markers[i].id < lowest_marker_id_)
        lowest_marker_id_ = temp_markers[i].id;
    }


    ROS_DEBUG_STREAM("The lowest Id marker " << lowest_marker_id_ );

    // Identify lowest marker ID with world's origin
    markers_[0].marker_id = lowest_marker_id_;

    markers_[0].geometry_msg_to_world.position.x = 0;
    markers_[0].geometry_msg_to_world.position.y = 0;
    markers_[0].geometry_msg_to_world.position.z = 0;

    markers_[0].geometry_msg_to_world.orientation.x = 0;
    markers_[0].geometry_msg_to_world.orientation.y = 0;
    markers_[0].geometry_msg_to_world.orientation.z = 0;
    markers_[0].geometry_msg_to_world.orientation.w = 1;

    // Relative position and Global position
    markers_[0].geometry_msg_to_previous.position.x = 0;
    markers_[0].geometry_msg_to_previous.position.y = 0;
    markers_[0].geometry_msg_to_previous.position.z = 0;

    markers_[0].geometry_msg_to_previous.orientation.x = 0;
    markers_[0].geometry_msg_to_previous.orientation.y = 0;
    markers_[0].geometry_msg_to_previous.orientation.z = 0;
    markers_[0].geometry_msg_to_previous.orientation.w = 1;

    // Transformation Pose to TF
    tf::Vector3 position;
    position.setX(0);
    position.setY(0);
    position.setZ(0);

    tf::Quaternion rotation;
    rotation.setX(0);
    rotation.setY(0);
    rotation.setZ(0);
    rotation.setW(1);

    markers_[0].tf_to_previous.setOrigin(position);
    markers_[0].tf_to_previous.setRotation(rotation);

    // Relative position of first marker equals Global position
    markers_[0].tf_to_world=markers_[0].tf_to_previous;

    // Increase count
    marker_counter_++;

    // Set sign of visibility of first marker
    markers_[0].visible=true;

    ROS_INFO_STREAM("First marker with ID: " << markers_[0].marker_id << " detected");

    //First marker does not have any previous marker
    markers_[0].previous_marker_id = THIS_IS_FIRST_MARKER;
  }

  //------------------------------------------------------
  // FOR EVERY MARKER DO
  //------------------------------------------------------
  std::string pkg_loc = ros::package::getPath("lidar_camera_calibration") + "/conf/transform.txt";
  std::ofstream outfile(pkg_loc.c_str(), std::ios_base::trunc);

  lidar_camera_calibration::marker_6dof marker_r_and_t;

  marker_r_and_t.header.stamp = ros::Time::now();
  marker_r_and_t.dof.data.clear();

  marker_r_and_t.num_of_markers = temp_markers.size();

  for(size_t i = 0; i < temp_markers.size();i++)
  {
    int index;
    int current_marker_id = temp_markers[i].id;

    //Draw marker convex, ID, cube and axis
    temp_markers[i].draw(output_image, cv::Scalar(0,0,255),2);
    aruco::CvDrawingUtils::draw3dCube(output_image,temp_markers[i], aruco_calib_params_);
    aruco::CvDrawingUtils::draw3dAxis(output_image,temp_markers[i], aruco_calib_params_);

    // Existing marker ?
    bool existing = false;
    int temp_counter = 0;

    while((existing == false) && (temp_counter < marker_counter_))
    {
      if(markers_[temp_counter].marker_id == current_marker_id)
      {
        index = temp_counter;
        existing = true;
        ROS_DEBUG_STREAM("Existing marker with ID: " << current_marker_id << "found");
      }
        temp_counter++;
    }

    //New marker ?
    if(existing == false)
    {
      index = marker_counter_;
      markers_[index].marker_id = current_marker_id;
      existing = true;
      ROS_DEBUG_STREAM("New marker with ID: " << current_marker_id << " found");
    }

    // Change visibility flag of new marker
    for(size_t j = 0;j < marker_counter_; j++)
    {
      for(size_t k = 0;k < temp_markers.size(); k++)
      {
        if(markers_[j].marker_id == temp_markers[k].id)
          markers_[j].visible = true;
      }
    }

    //------------------------------------------------------
    // For existing marker do
    //------------------------------------------------------
    if((index < marker_counter_) && (first_marker_detected_ == true))
    {
      outfile << temp_markers[i] << "\n";
      marker_r_and_t.dof.data.push_back( temp_markers[i].id ); 
      marker_r_and_t.dof.data.push_back( temp_markers[i].Tvec.ptr<float>(0)[0] );
      marker_r_and_t.dof.data.push_back( temp_markers[i].Tvec.ptr<float>(0)[1] );
      marker_r_and_t.dof.data.push_back( temp_markers[i].Tvec.ptr<float>(0)[2] );
      marker_r_and_t.dof.data.push_back( temp_markers[i].Rvec.ptr<float>(0)[0] );
      marker_r_and_t.dof.data.push_back( temp_markers[i].Rvec.ptr<float>(0)[1] );
      marker_r_and_t.dof.data.push_back( temp_markers[i].Rvec.ptr<float>(0)[2] );

      markers_[index].current_camera_tf = arucoMarker2Tf(temp_markers[i]);
      markers_[index].current_camera_tf = markers_[index].current_camera_tf.inverse();

      const tf::Vector3 marker_origin = markers_[index].current_camera_tf.getOrigin();
      markers_[index].current_camera_pose.position.x = marker_origin.getX();
      markers_[index].current_camera_pose.position.y = marker_origin.getY();
      markers_[index].current_camera_pose.position.z = marker_origin.getZ();

      const tf::Quaternion marker_quaternion = markers_[index].current_camera_tf.getRotation();
      markers_[index].current_camera_pose.orientation.x = marker_quaternion.getX();
      markers_[index].current_camera_pose.orientation.y = marker_quaternion.getY();
      markers_[index].current_camera_pose.orientation.z = marker_quaternion.getZ();
      markers_[index].current_camera_pose.orientation.w = marker_quaternion.getW();
    }

    //------------------------------------------------------
    // For new marker do
    //------------------------------------------------------
    if((index == marker_counter_) && (first_marker_detected_ == true))
    {
      outfile << temp_markers[i] << "\n";
      marker_r_and_t.dof.data.push_back( temp_markers[i].id ); 
      marker_r_and_t.dof.data.push_back( temp_markers[i].Tvec.ptr<float>(0)[0] );
      marker_r_and_t.dof.data.push_back( temp_markers[i].Tvec.ptr<float>(0)[1] );
      marker_r_and_t.dof.data.push_back( temp_markers[i].Tvec.ptr<float>(0)[2] );
      marker_r_and_t.dof.data.push_back( temp_markers[i].Rvec.ptr<float>(0)[0] );
      marker_r_and_t.dof.data.push_back( temp_markers[i].Rvec.ptr<float>(0)[1] );
      marker_r_and_t.dof.data.push_back( temp_markers[i].Rvec.ptr<float>(0)[2] );

      markers_[index].current_camera_tf=arucoMarker2Tf(temp_markers[i]);

      tf::Vector3 marker_origin = markers_[index].current_camera_tf.getOrigin();
      markers_[index].current_camera_pose.position.x = marker_origin.getX();
      markers_[index].current_camera_pose.position.y = marker_origin.getY();
      markers_[index].current_camera_pose.position.z = marker_origin.getZ();

      tf::Quaternion marker_quaternion = markers_[index].current_camera_tf.getRotation();
      markers_[index].current_camera_pose.orientation.x = marker_quaternion.getX();
      markers_[index].current_camera_pose.orientation.y = marker_quaternion.getY();
      markers_[index].current_camera_pose.orientation.z = marker_quaternion.getZ();
      markers_[index].current_camera_pose.orientation.w = marker_quaternion.getW();

      // Naming - TFs
      std::stringstream camera_tf_id;
      std::stringstream camera_tf_id_old;
      std::stringstream marker_tf_id_old;

      camera_tf_id << "camera_" << index;

      // Flag to keep info if any_known marker_visible in actual image
      bool any_known_marker_visible = false;

      // Array ID of markers, which position of new marker is calculated
      int last_marker_id;

      // Testing, if is possible calculate position of a new marker to old known marker
      for(int k = 0; k < index; k++)
      {
        if((markers_[k].visible == true) && (any_known_marker_visible == false))
        {
          if(markers_[k].previous_marker_id != -1)
          {
            any_known_marker_visible = true;
            camera_tf_id_old << "camera_" << k;
            marker_tf_id_old << "marker_" << k;
            markers_[index].previous_marker_id = k;
            last_marker_id = k;
           }
         }
       }

     // New position can be calculated
     if(any_known_marker_visible == true)
     {
       // Generating TFs for listener
       for(char k = 0; k < 10; k++)
       {
         // TF from old marker and its camera
         broadcaster_.sendTransform(tf::StampedTransform(markers_[last_marker_id].current_camera_tf,ros::Time::now(),
                                                         marker_tf_id_old.str(),camera_tf_id_old.str()));

         // TF from old camera to new camera
         broadcaster_.sendTransform(tf::StampedTransform(markers_[index].current_camera_tf,ros::Time::now(),
                                                         camera_tf_id_old.str(),camera_tf_id.str()));

         ros::Duration(BROADCAST_WAIT_INTERVAL).sleep();
       }

        // Calculate TF between two markers
        listener_->waitForTransform(marker_tf_id_old.str(),camera_tf_id.str(),ros::Time(0),
                                    ros::Duration(WAIT_FOR_TRANSFORM_INTERVAL));
        try
        {
          broadcaster_.sendTransform(tf::StampedTransform(markers_[last_marker_id].current_camera_tf,ros::Time::now(),
                                                          marker_tf_id_old.str(),camera_tf_id_old.str()));

          broadcaster_.sendTransform(tf::StampedTransform(markers_[index].current_camera_tf,ros::Time::now(),
                                                          camera_tf_id_old.str(),camera_tf_id.str()));

          listener_->lookupTransform(marker_tf_id_old.str(),camera_tf_id.str(),ros::Time(0),
                                     markers_[index].tf_to_previous);
        }
        catch(tf::TransformException &e)
        {
          ROS_ERROR("Not able to lookup transform");
        }

        // Save origin and quaternion of calculated TF
        marker_origin = markers_[index].tf_to_previous.getOrigin();
        marker_quaternion = markers_[index].tf_to_previous.getRotation();

        // If plane type selected roll, pitch and Z axis are zero
        if(space_type_ == "plane")
        {
          double roll, pitch, yaw;
          tf::Matrix3x3(marker_quaternion).getRPY(roll,pitch,yaw);
          roll = 0;
          pitch = 0;
          marker_origin.setZ(0);
          marker_quaternion.setRPY(pitch,roll,yaw);
        }

        markers_[index].tf_to_previous.setRotation(marker_quaternion);
        markers_[index].tf_to_previous.setOrigin(marker_origin);

        marker_origin = markers_[index].tf_to_previous.getOrigin();
        markers_[index].geometry_msg_to_previous.position.x = marker_origin.getX();
        markers_[index].geometry_msg_to_previous.position.y = marker_origin.getY();
        markers_[index].geometry_msg_to_previous.position.z = marker_origin.getZ();

        marker_quaternion = markers_[index].tf_to_previous.getRotation();
        markers_[index].geometry_msg_to_previous.orientation.x = marker_quaternion.getX();
        markers_[index].geometry_msg_to_previous.orientation.y = marker_quaternion.getY();
        markers_[index].geometry_msg_to_previous.orientation.z = marker_quaternion.getZ();
        markers_[index].geometry_msg_to_previous.orientation.w = marker_quaternion.getW();

        // increase marker count
        marker_counter_++;

        // Invert and position of new marker to compute camera pose above it
        markers_[index].current_camera_tf = markers_[index].current_camera_tf.inverse();

        marker_origin = markers_[index].current_camera_tf.getOrigin();
        markers_[index].current_camera_pose.position.x = marker_origin.getX();
        markers_[index].current_camera_pose.position.y = marker_origin.getY();
        markers_[index].current_camera_pose.position.z = marker_origin.getZ();

        marker_quaternion = markers_[index].current_camera_tf.getRotation();
        markers_[index].current_camera_pose.orientation.x = marker_quaternion.getX();
        markers_[index].current_camera_pose.orientation.y = marker_quaternion.getY();
        markers_[index].current_camera_pose.orientation.z = marker_quaternion.getZ();
        markers_[index].current_camera_pose.orientation.w = marker_quaternion.getW();

        // Publish all TFs and markers
        publishTfs(false);
      }
    }

    //------------------------------------------------------
    // Compute global position of new marker
    //------------------------------------------------------
    if((marker_counter_previous_ < marker_counter_) && (first_marker_detected_ == true))
    {
      // Publish all TF five times for listener
      for(char k = 0; k < 5; k++)
        publishTfs(false);

      std::stringstream marker_tf_name;
      marker_tf_name << "marker_" << index;

      listener_->waitForTransform("world",marker_tf_name.str(),ros::Time(0),
                                  ros::Duration(WAIT_FOR_TRANSFORM_INTERVAL));
      try
      {
        listener_->lookupTransform("world",marker_tf_name.str(),ros::Time(0),
                                   markers_[index].tf_to_world);
      }
      catch(tf::TransformException &e)
      {
        ROS_ERROR("Not able to lookup transform");
      }

      // Saving TF to Pose
      const tf::Vector3 marker_origin = markers_[index].tf_to_world.getOrigin();
      markers_[index].geometry_msg_to_world.position.x = marker_origin.getX();
      markers_[index].geometry_msg_to_world.position.y = marker_origin.getY();
      markers_[index].geometry_msg_to_world.position.z = marker_origin.getZ();

      tf::Quaternion marker_quaternion=markers_[index].tf_to_world.getRotation();
      markers_[index].geometry_msg_to_world.orientation.x = marker_quaternion.getX();
      markers_[index].geometry_msg_to_world.orientation.y = marker_quaternion.getY();
      markers_[index].geometry_msg_to_world.orientation.z = marker_quaternion.getZ();
      markers_[index].geometry_msg_to_world.orientation.w = marker_quaternion.getW();
    }
  }
  outfile.close();
  lidar_camera_calibration_rt.publish(marker_r_and_t);
  marker_r_and_t.dof.data.clear();
  //------------------------------------------------------
  // Compute which of visible markers is the closest to the camera
  //------------------------------------------------------
  bool any_markers_visible=false;
  int num_of_visible_markers=0;

  if(first_marker_detected_ == true)
  {
    double minimal_distance = INIT_MIN_SIZE_VALUE;
    for(int k = 0; k < num_of_markers_; k++)
    {
      double a,b,c,size;

      // If marker is visible, distance is calculated
      if(markers_[k].visible==true)
      {
        a = markers_[k].current_camera_pose.position.x;
        b = markers_[k].current_camera_pose.position.y;
        c = markers_[k].current_camera_pose.position.z;
        size = std::sqrt((a * a) + (b * b) + (c * c));
        if(size < minimal_distance)
        {
          minimal_distance = size;
          closest_camera_index_ = k;
        }

        any_markers_visible = true;
        num_of_visible_markers++;
      }
    }
  }

  //------------------------------------------------------
  // Publish all known markers
  //------------------------------------------------------
  if(first_marker_detected_ == true)
    publishTfs(true);

  //------------------------------------------------------
  // Compute global camera pose
  //------------------------------------------------------
  if((first_marker_detected_ == true) && (any_markers_visible == true))
  {
    std::stringstream closest_camera_tf_name;
    closest_camera_tf_name << "camera_" << closest_camera_index_;

    listener_->waitForTransform("world",closest_camera_tf_name.str(),ros::Time(0),
                                ros::Duration(WAIT_FOR_TRANSFORM_INTERVAL));
    try
    {
      listener_->lookupTransform("world",closest_camera_tf_name.str(),ros::Time(0),
                                 world_position_transform_);
    }
    catch(tf::TransformException &ex)
    {
      ROS_ERROR("Not able to lookup transform");
    }

    // Saving TF to Pose
    const tf::Vector3 marker_origin = world_position_transform_.getOrigin();
    world_position_geometry_msg_.position.x = marker_origin.getX();
    world_position_geometry_msg_.position.y = marker_origin.getY();
    world_position_geometry_msg_.position.z = marker_origin.getZ();

    tf::Quaternion marker_quaternion = world_position_transform_.getRotation();
    world_position_geometry_msg_.orientation.x = marker_quaternion.getX();
    world_position_geometry_msg_.orientation.y = marker_quaternion.getY();
    world_position_geometry_msg_.orientation.z = marker_quaternion.getZ();
    world_position_geometry_msg_.orientation.w = marker_quaternion.getW();
  }

  //------------------------------------------------------
  // Publish all known markers
  //------------------------------------------------------
  if(first_marker_detected_ == true)
    publishTfs(true);

  //------------------------------------------------------
  // Publish custom marker message
  //------------------------------------------------------
  aruco_mapping::ArucoMarker marker_msg;

  if((any_markers_visible == true))
  {
    marker_msg.header.stamp = ros::Time::now();
    marker_msg.header.frame_id = "world";
    marker_msg.marker_visibile = true;
    marker_msg.num_of_visible_markers = num_of_visible_markers;
    marker_msg.global_camera_pose = world_position_geometry_msg_;
    marker_msg.marker_ids.clear();
    marker_msg.global_marker_poses.clear();
    for(size_t j = 0; j < marker_counter_; j++)
    {
      if(markers_[j].visible == true)
      {
        marker_msg.marker_ids.push_back(markers_[j].marker_id);
        marker_msg.global_marker_poses.push_back(markers_[j].geometry_msg_to_world);       
      }
    }
  }
  else
  {
    marker_msg.header.stamp = ros::Time::now();
    marker_msg.header.frame_id = "world";
    marker_msg.num_of_visible_markers = num_of_visible_markers;
    marker_msg.marker_visibile = false;
    marker_msg.marker_ids.clear();
    marker_msg.global_marker_poses.clear();
  }

  // Publish custom marker msg
  marker_msg_pub_.publish(marker_msg);

  return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////

void
ArucoMapping::publishTfs(bool world_option)
{
  for(int i = 0; i < marker_counter_; i++)
  {
    // Actual Marker
    std::stringstream marker_tf_id;
    marker_tf_id << "marker_" << i;
    // Older marker - or World
    std::stringstream marker_tf_id_old;
    if(i == 0)
      marker_tf_id_old << "world";
    else
      marker_tf_id_old << "marker_" << markers_[i].previous_marker_id;
    broadcaster_.sendTransform(tf::StampedTransform(markers_[i].tf_to_previous,ros::Time::now(),marker_tf_id_old.str(),marker_tf_id.str()));

    // Position of camera to its marker
    std::stringstream camera_tf_id;
    camera_tf_id << "camera_" << i;
    broadcaster_.sendTransform(tf::StampedTransform(markers_[i].current_camera_tf,ros::Time::now(),marker_tf_id.str(),camera_tf_id.str()));

    if(world_option == true)
    {
      // Global position of marker TF
      std::stringstream marker_globe;
      marker_globe << "marker_globe_" << i;
      broadcaster_.sendTransform(tf::StampedTransform(markers_[i].tf_to_world,ros::Time::now(),"world",marker_globe.str()));
    }

    // Cubes for RVIZ - markers
    publishMarker(markers_[i].geometry_msg_to_previous,markers_[i].marker_id,i);
  }

  // Global Position of object
  if(world_option == true)
    broadcaster_.sendTransform(tf::StampedTransform(world_position_transform_,ros::Time::now(),"world","camera_position"));
}

////////////////////////////////////////////////////////////////////////////////////////////////

void
ArucoMapping::publishMarker(geometry_msgs::Pose marker_pose, int marker_id, int index)
{
  visualization_msgs::Marker vis_marker;

  if(index == 0)
    vis_marker.header.frame_id = "world";
  else
  {
    std::stringstream marker_tf_id_old;
    marker_tf_id_old << "marker_" << markers_[index].previous_marker_id;
    vis_marker.header.frame_id = marker_tf_id_old.str();
  }

  vis_marker.header.stamp = ros::Time::now();
  vis_marker.ns = "basic_shapes";
  vis_marker.id = marker_id;
  vis_marker.type = visualization_msgs::Marker::CUBE;
  vis_marker.action = visualization_msgs::Marker::ADD;

  vis_marker.pose = marker_pose;
  vis_marker.scale.x = marker_size_;
  vis_marker.scale.y = marker_size_;
  vis_marker.scale.z = RVIZ_MARKER_HEIGHT;

  vis_marker.color.r = RVIZ_MARKER_COLOR_R;
  vis_marker.color.g = RVIZ_MARKER_COLOR_G;
  vis_marker.color.b = RVIZ_MARKER_COLOR_B;
  vis_marker.color.a = RVIZ_MARKER_COLOR_A;

  vis_marker.lifetime = ros::Duration(RVIZ_MARKER_LIFETIME);

  marker_visualization_pub_.publish(vis_marker);
}

////////////////////////////////////////////////////////////////////////////////////////////////

tf::Transform
ArucoMapping::arucoMarker2Tf(const aruco::Marker &marker)
{
  cv::Mat marker_rotation(3,3, CV_32FC1);
  cv::Rodrigues(marker.Rvec, marker_rotation);
  cv::Mat marker_translation = marker.Tvec;

  cv::Mat rotate_to_ros(3,3,CV_32FC1);
  rotate_to_ros.at<float>(0,0) = -1.0;
  rotate_to_ros.at<float>(0,1) = 0;
  rotate_to_ros.at<float>(0,2) = 0;
  rotate_to_ros.at<float>(1,0) = 0;
  rotate_to_ros.at<float>(1,1) = 0;
  rotate_to_ros.at<float>(1,2) = 1.0;
  rotate_to_ros.at<float>(2,0) = 0.0;
  rotate_to_ros.at<float>(2,1) = 1.0;
  rotate_to_ros.at<float>(2,2) = 0.0;

  marker_rotation = marker_rotation * rotate_to_ros.t();

  // Origin solution
  tf::Matrix3x3 marker_tf_rot(marker_rotation.at<float>(0,0),marker_rotation.at<float>(0,1),marker_rotation.at<float>(0,2),
                              marker_rotation.at<float>(1,0),marker_rotation.at<float>(1,1),marker_rotation.at<float>(1,2),
                              marker_rotation.at<float>(2,0),marker_rotation.at<float>(2,1),marker_rotation.at<float>(2,2));

  tf::Vector3 marker_tf_tran(marker_translation.at<float>(0,0),
                             marker_translation.at<float>(1,0),
                             marker_translation.at<float>(2,0));

  /*ROS_INFO_STREAM("in aruco mapping: \n" <<  marker << "\n");
  ROS_INFO_STREAM("Edited: \n" <<  marker_tf_tran[0] << "\n");*/
  return tf::Transform(marker_tf_rot, marker_tf_tran);


}



}  //aruco_mapping

#endif  //ARUCO_MAPPING_CPP
