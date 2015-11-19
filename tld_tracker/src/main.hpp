/*  Copyright 2012 UdeS University of Sherbrooke
 *
 *   This file is part of ROS_OpenTLD.
 *
 *   ROS_OpenTLD is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *
 *   ROS_OpenTLD is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with ROS_OpenTLD. If not, see <http://www.gnu.org/licenses/>.
 *
 */
/*
 * main.hpp
 *
 *  Created on: June 8, 2012
 *      Author: Ronan Chauvin
 */

//* Main
/**
* State machine of OpenTLD
* INIT
* TRACKER_INIT
* TRACKING
* TRACKING_FALSE_POSITIVE
* STOPPED
*
*/

#ifndef MAIN_HPP_
#define MAIN_HPP_

#include <tld/TLD.h>
#include <boost/interprocess/sync/interprocess_mutex.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>

#include <ros/ros.h>

#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Char.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Header.h>
#include <tld_msgs/BoundingBox.h>
#include <tld_msgs/Target.h>

// TF libraries
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <string>

// Parametri camera
#include <tld_tracker/depth_traits.h> // 'depth_image_proc'
#include <image_geometry/pinhole_camera_model.h>


class Main
{
public:
  Main()
  {
    tld = new tld::TLD();
    state = INIT;

    ros::NodeHandle np("~");
    np.param("showOutput", showOutput, true);
    np.param("loadModel", loadModel, false);
    np.param("autoFaceDetection", autoFaceDetection, false);
    np.param("exportModelAfterRun", exportModelAfterRun, false);
    np.param("modelImportFile", modelImportFile, std::string("model"));
    np.param("modelExportFile", modelExportFile, std::string("model"));
    np.param("cascadePath", face_cascade_path, std::string("haarcascade_frontalface_alt.xml"));

    np.param("x", target_bb.x, 100);
    np.param("y", target_bb.y, 100);
    np.param("width", target_bb.width, 100);
    np.param("height", target_bb.height, 100);
    np.param("correctBB", correctBB, false);

    pub1 = n.advertise<tld_msgs::BoundingBox>("tld_tracked_object", 1000, true);
    pub2 = n.advertise<std_msgs::Float32>("tld_fps", 1000, true);
    sub1 = n.subscribe("image", 1000, &Main::imageReceivedCB, this);
    sub2 = n.subscribe("bounding_box", 1000, &Main::targetReceivedCB, this);
    sub3 = n.subscribe("cmds", 1000, &Main::cmdReceivedCB, this);

    sub_kalman = n.subscribe("odometry/filtered", 1000, &Main::kalmanOdomReceivedCB, this);
    // Callback per ottenere i parametri intrinseci della camera, dato che il modello della camera
    // è costante questa callback verrà eseguita una sola volta.
    depth_camera_info_sub = n.subscribe("/kinect2/qhd/camera_info", 1000, &Main::depthCameraInfoCb, this);
    tf::TransformListener transform_listener;

    odom_bb.x = -1;
    odom_bb.y = -1;
    odom_bb.width = -1;
    odom_bb.height = -1;

    pub_odom_rect = n.advertise<tld_msgs::BoundingBox>("odom_bb", 1000, true);
    pub_redirected_image = n.advertise<sensor_msgs::Image>("redirected_image_gui", 1000, true);

    start_time = ros::Time::now();
    last_tld_curr_bb_width = 36;
    last_tld_curr_bb_height = 36;

    semaphore.lock();
  }

  ~Main()
  {
    delete tld;
  }

  void process();

private:
  tld::TLD* tld;
  bool showOutput;
  bool exportModelAfterRun;
  bool loadModel;
  bool autoFaceDetection;
  std::string modelImportFile;
  std::string modelExportFile;

  bool kalman_has_converged;
  bool out_of_view;
  ros::Time start_time;

  // Camera parameters
  image_geometry::PinholeCameraModel cam_model_;
  bool camera_info_received;
  bool is_false_positive;
  double min_confidence;
  double point_max_height;
  double unit_scaling;
  double z_thresh;
  float center_x, center_y;
  float constant_x, constant_y;
  int img_height, img_width;
  std::string cam_frame_id;

  int tld_curr_bb_width;
  int tld_curr_bb_height;
  int last_tld_curr_bb_width;
  int last_tld_curr_bb_height;


  tld_msgs::BoundingBox odom_bb;

  enum
  {
    INIT,
    TRACKER_INIT,
    TRACKING,
    STOPPED
  } state;

  bool correctBB;
  cv::Mat target_image;
  cv::Rect target_bb;

  std_msgs::Header img_header;
  cv::Mat img;
  cv_bridge::CvImagePtr img_buffer_ptr;
  cv::Mat gray;
  boost::interprocess::interprocess_mutex mutex;
  boost::interprocess::interprocess_mutex semaphore;
  ros::NodeHandle n;
  ros::Publisher pub1;
  ros::Publisher pub2;
  ros::Subscriber sub1;
  ros::Subscriber sub2;
  ros::Subscriber sub3;
  ros::Subscriber depth_camera_info_sub;

  ros::Publisher pub_odom_rect;
  ros::Publisher pub_redirected_image;
  ros::Subscriber sub_kalman;

  tf::TransformListener transform_listener;

  std::string face_cascade_path;
  cv::CascadeClassifier face_cascade;

  /*!
  * \brief This function return a new image has been received
  */
  bool newImageReceived();
  void getLastImageFromBuffer();

  /*!
  * \brief ROS image callback.
  */
  void imageReceivedCB(const sensor_msgs::ImageConstPtr& msg);

  /*!
  * \brief ROS target callback.
  */
  void targetReceivedCB(const tld_msgs::TargetConstPtr& msg);

  /*!
  * \brief ROS command callback.
  */
  void cmdReceivedCB(const std_msgs::CharConstPtr& cmd);

  /*!
  * \brief ROS command callback.
  */
  void kalmanOdomReceivedCB(const nav_msgs::OdometryConstPtr& cmd);

  /*!
  * \brief ROS command callback.
  */
  void depthCameraInfoCb(const sensor_msgs::CameraInfo::ConstPtr& depth_camera_info);

  /*!
  * \brief ROS command callback.
  */
  void odomToImgPlan(const double odom_x, const double odom_y, const double odom_z, int& img_x, int& img_y);

  /*!
  * \brief This function sends the tracked object as a BoudingBox message.
  *
  * \param x
  * \param y
  * \param width
  * \param height
  * \param confidence
  */
  void sendTrackedObject(int x, int y, int width, int height, float conf);

  void clearBackground();
  void stopTracking();
  void toggleLearning();
  void alternatingMode();
  void exportModel();
  void importModel();
  void reset();

  /*!
  * \brief This function allows to automatically initialize the target
  * using the OpenCV Haar-cascade detector.
  */
  cv::Rect faceDetection();
};

#endif /* MAIN_HPP_ */
