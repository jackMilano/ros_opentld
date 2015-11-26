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
 * main.cpp
 *
 *  Created on: June 8, 2012
 *      Author: Ronan Chauvin
 */

#include "main.hpp"

#include <ros/time.h>
#include <ros/duration.h>

#include <sensor_msgs/image_encodings.h>


#define WAIT_TRANS_TIME 5.0f // Tempo di attesa per la trasformata 'world-->odom'


namespace enc = sensor_msgs::image_encodings;

void Main::process()
{
  if(autoFaceDetection && !face_cascade.load(face_cascade_path))
  {
    ROS_FATAL("--(!)Error loading cascade detector\n");
    return;
  }

  while(ros::ok())
  {
    switch(state)
    {
    case INIT:
      if(newImageReceived())
      {
        if(showOutput)
        {
          sendTrackedObject(0, 0, 0, 0, 0.0);
        }

        getLastImageFromBuffer();

        tld->detectorCascade->imgWidth = gray.cols;
        tld->detectorCascade->imgHeight = gray.rows;
        tld->detectorCascade->imgWidthStep = gray.step;

        ROS_DEBUG("'gray.cols' = %d.", gray.cols);
        ROS_DEBUG("'gray.rows' = %d.", gray.rows);

        state = TRACKER_INIT;
      }

      break;

    case TRACKER_INIT:
      if(loadModel && !modelImportFile.empty())
      {
        ROS_INFO("Loading model %s", modelImportFile.c_str());

        tld->readFromFile(modelImportFile.c_str());
        tld->learningEnabled = false;
        state = TRACKING;
      }
      else if(autoFaceDetection || correctBB)
      {
        if(autoFaceDetection)
        {
          target_image = gray;
          target_bb = faceDetection();
        }

        sendTrackedObject(target_bb.x, target_bb.y, target_bb.width, target_bb.height, 1.0);

        ROS_INFO("ros_opentld: Starting at %d %d %d %d\n", target_bb.x, target_bb.y, target_bb.width, target_bb.height);

        // 'tld->selectObject' chiama 'detectorCascade->release()', 'detectorCascade->init()'
        // ed infine 'initialLearning()'
        tld->selectObject(target_image, &target_bb);
        tld->learningEnabled = true;
        state = TRACKING;
      }
      else
      {
        ros::Duration(1.0).sleep();
        ROS_INFO("Waiting for a BB");
      }

      break;

    case TRACKING:
      if(newImageReceived())
      {
        ros::Time tic = ros::Time::now();

        getLastImageFromBuffer();
        tld->processImage(img);

        ros::Duration toc = (ros::Time::now() - tic);
        float fps = 1.0 / toc.toSec();

        std_msgs::Float32 msg_fps;
        msg_fps.data = fps;
        pub2.publish(msg_fps);

        if(showOutput)
        {
          if(tld->currBB != NULL)
          {
            tld_curr_bb_width = tld->currBB->width;
            tld_curr_bb_height = tld->currBB->height;
            sendTrackedObject(tld->currBB->x, tld->currBB->y, tld->currBB->width, tld->currBB->height, tld->currConf);
          }
          else
          {
            sendTrackedObject(1, 1, 1, 1, 0.0);
          }
        }
      }

      break;

    case STOPPED:
      ros::Duration(1.0).sleep();
      ROS_INFO("Tracker stopped");
      break;

    default:
      break;
    }
  }

  if(exportModelAfterRun)
  {
    tld->writeToFile(modelExportFile.c_str());
  }

  semaphore.unlock();

  return;
}


void Main::imageReceivedCB(const sensor_msgs::ImageConstPtr& msg)
{
  bool empty = false;
  mutex.lock();

  if(img_buffer_ptr.get() == 0)
  {
    empty = true;
  }

  try
  {
    if(enc::isColor(msg->encoding))
    {
      img_buffer_ptr = cv_bridge::toCvCopy(msg, enc::RGB8);
    }
    else
    {
      img_buffer_ptr = cv_bridge::toCvCopy(msg, enc::MONO8);
      cv::cvtColor(img_buffer_ptr->image, img_buffer_ptr->image, CV_GRAY2BGR);
    }
  }
  catch(cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  if(empty)
  {
    semaphore.unlock();
  }

  mutex.unlock();

  return;
}

//XXX: ogni volta che viene chiamata questa callback viene chiamato anche 'reset()'!
void Main::targetReceivedCB(const tld_msgs::TargetConstPtr& msg)
{
  // Senza il reset non funziona la reinizializzazione.
  // XXX: è possibile chiamare solo alcune delle funzioni che vengono chiamate da reset?
  reset();

  ROS_ASSERT(msg->bb.x >= 0);
  ROS_ASSERT(msg->bb.y >= 0);
  ROS_ASSERT(msg->bb.width > 0);
  ROS_ASSERT(msg->bb.height > 0);
  ROS_INFO("ros_opentld: Bounding Box received. x = %d, y = %d, width = %d, height = %d.", msg->bb.x, msg->bb.y,
           msg->bb.width, msg->bb.height);

  target_bb.x = msg->bb.x;
  target_bb.y = msg->bb.y;
  target_bb.width = msg->bb.width;
  target_bb.height = msg->bb.height;

  try
  {
    target_image = cv_bridge::toCvCopy(msg->img, enc::MONO8)->image;
  }
  catch(cv_bridge::Exception& e)
  {
    ROS_ERROR("ros_opentld: cv_bridge exception: %s", e.what());
    return;
  }

  correctBB = true;

  return;
}

void Main::cmdReceivedCB(const std_msgs::CharConstPtr& cmd)
{

  switch(cmd->data)
  {
  case 'b':
    clearBackground();
    break;

  case 'c':
    stopTracking();
    break;

  case 'l':
    toggleLearning();
    break;

  case 'a':
    alternatingMode();
    break;

  case 'e':
    exportModel();
    break;

  case 'i':
    importModel();
    break;

  case 'r':
    reset();
    break;

  default:
    break;
  }

  return;
}


void Main::depthCameraInfoCb(const sensor_msgs::CameraInfo::ConstPtr& depth_camera_info)
{
  // Load the camera model structure with the pinhole model (Intrinsic + distortion coefficients)
  // of the IR camera (depth).
  cam_model_.fromCameraInfo(depth_camera_info);

  cam_frame_id = depth_camera_info->header.frame_id;

  ROS_INFO("Depth Camera Model Loaded");

  // Do it once since the camera model is constant
  depth_camera_info_sub.shutdown();

  ROS_INFO("Camera Info subscriber shut down");

  // Le dimensioni dell'immagine servono a capire se ci si trova in un "out-of-view".
  img_width = depth_camera_info->width;
  img_height = depth_camera_info->height;

  // Use correct principal point from calibration
  center_x = cam_model_.cx();
  center_y = cam_model_.cy();

  // Combine unit conversion (if necessary) with scaling by focal length for computing (X,Y)
  unit_scaling = depth_image_proc::DepthTraits<uint16_t>::toMeters(uint16_t(1));
  constant_x = unit_scaling / cam_model_.fx();
  constant_y = unit_scaling / cam_model_.fy();

  camera_info_received = true;

  ROS_INFO("depthCameraInfoCb end.");

  return;
}

// La funzione proietta in 2D le coordinate 3D dell'odometria.
// XXX: si da per scontato che il sistema di riferimento sia lo stesso e sia quello della camera.
void Main::odomToImgPlan(const double odom_x, const double odom_y, const double odom_z, int& img_x, int& img_y)
{
  const uint16_t p_depth = depth_image_proc::DepthTraits<uint16_t>::fromMeters(odom_z);

  img_x = static_cast<int>(odom_x / (p_depth * constant_x) + center_x);
  img_y = static_cast<int>(odom_y / (p_depth * constant_y) + center_y);

  return;
}

// Riceve l'odometria di kalman e la converte in 2D sul piano
void Main::kalmanOdomReceivedCB(const nav_msgs::OdometryConstPtr& kalman_odom_msg)
{
  //ROS_INFO("ros_tld_tracker: odometria kalman ricevuta");

  //mutex.lock();
  //semaphore.lock();

  // Non appena è disponibile la transform tra il sistema di riferimento dell'odometria e
  // quello della camera (deve essere già attivo `robot_localization`), si procede all'operazione di
  // conversione della posizione 3D dell'odometria dal primo sistema di riferimento all'altro.
  if(!transform_listener.waitForTransform(kalman_odom_msg->header.frame_id, cam_frame_id, kalman_odom_msg->header.stamp,
                                          ros::Duration(WAIT_TRANS_TIME)))
  {
    ROS_ERROR("ros_open_tld: wait for transform %s --> %s timed out!!", kalman_odom_msg->header.frame_id.c_str(),
              cam_frame_id.c_str());
    return;
  }

  // I primi 3 secondi diamo il tempo a Kalman di convergere
  if(!kalman_has_converged)
  {
    ros::Duration time_passed = ros::Time::now() - start_time;

    if(time_passed > ros::Duration(3.0))
    {
      kalman_has_converged = true;
      ROS_INFO("Sono passati 3 secondi!!");
    }
  }

  geometry_msgs::PoseStamped odom_pose;
  odom_pose.header = kalman_odom_msg->header; // frame_id = "kalman_odom_msg"
  odom_pose.pose = kalman_odom_msg->pose.pose;
  geometry_msgs::PoseStamped cam_odom; // frame_id = "kinect2_rgb_optical_frame"
  transform_listener.transformPose(cam_frame_id, odom_pose, cam_odom);

  // Calcolo posizione sul piano immagine (sistema di riferimento camera) dell'odometria.
  int odom_img_x = 0, odom_img_y = 0;
  odomToImgPlan(cam_odom.pose.position.x, cam_odom.pose.position.y, cam_odom.pose.position.z, odom_img_x, odom_img_y);

  // PUBBLICAZIONE DELLA BOUNDING BOX CORRISPONDENTE ALLA POSIZIONE DELL'ODOMETRIA:
  // - serve per i test
  // - la bounding box viene calcolata come se avesse il centro nella posizione dell'odometria
  //   (2D, sistema di riferimento camera)
  // - la larghezza e l'altezza della bounding box corrispondono a quelle dell'ultima bounding box valida
  odom_bb.header.seq = img_header.seq;
  odom_bb.header.stamp = ros::Time::now();
  odom_bb.header.frame_id = cam_frame_id;

  int tmp_tld_currbb_width;

  if(tld_curr_bb_width != 1)
  {
    tmp_tld_currbb_width = tld_curr_bb_width;
    last_tld_curr_bb_width = tld_curr_bb_width;
  }
  else
  {
    if(last_tld_curr_bb_width > 1)
    {
      tmp_tld_currbb_width = last_tld_curr_bb_width;
    }
    else
    {
      tmp_tld_currbb_width = 36;
    }
  }

  int tmp_tld_currbb_height;

  if(tld_curr_bb_height != 1)
  {
    tmp_tld_currbb_height = tld_curr_bb_height;
    last_tld_curr_bb_height = tld_curr_bb_height;
  }
  else
  {
    if(last_tld_curr_bb_height > 1)
    {
      tmp_tld_currbb_height = last_tld_curr_bb_height;
    }
    else
    {
      tmp_tld_currbb_height = 36;
    }
  }

  odom_bb.x = odom_img_x - static_cast<int>(round(static_cast<double>(tld_curr_bb_width) / 2.0));

  if(odom_bb.x < 0)
  {
    ROS_WARN("Attenzione! La coordinata 'x' della Bounding Box dell'odometria calcolata è negativa. Viene azzerata.");
    odom_bb.x = 0;
  }

  odom_bb.y = odom_img_y - static_cast<int>(round(static_cast<double>(tld_curr_bb_height) / 2.0));

  if(odom_bb.y < 0)
  {
    ROS_WARN("Attenzione! La coordinata 'y' della Bounding Box dell'odometria calcolata è negativa. Viene azzerata.");
    odom_bb.y = 0;
  }

  odom_bb.width = tld_curr_bb_width;
  odom_bb.height = tld_curr_bb_height;

  odom_bb.confidence = 1.0f;

  if((odom_img_x < 0) || (odom_img_x + odom_bb.width - 2) > img_width || (odom_img_y < 0)
      || (odom_img_y + odom_bb.height - 2) > img_height)
  {
    if(odom_img_x < 0)
    {
      ROS_WARN("Out of view. odom_img_x = %d", odom_img_x);
    }

    if((odom_img_x + tld_curr_bb_width - 2) > img_width)
    {
      ROS_WARN("Out of view. odom_img_x = %d, img_width = %d, tld_currbb_width = %d", odom_img_x, img_width,
               tld_curr_bb_width);
    }

    if(odom_img_y < 0)
    {
      ROS_WARN("Out of view. odom_img_y = %d", odom_img_y);
    }

    if((odom_img_y + tld_curr_bb_height - 2) > img_height)
    {
      ROS_WARN("Out of view. odom_img_y = %d, img_height = %d, tld_currbb_height = %d", odom_img_y, img_height,
               tld_curr_bb_height);
    }

    out_of_view = true;
  }
  else
  {
    out_of_view = false;
  }

  //ROS_INFO("ros_tld_tracker: bounding box odometria inviata");
  pub_odom_rect.publish(odom_bb);

  //mutex.unlock();
  //semaphore.unlock();

  return;
}

void Main::sendTrackedObject(int x, int y, int width, int height, float confidence)
{
  tld_msgs::BoundingBox msg;
  msg.header = img_header; //Add the Header of the last image processed
  msg.x = x;
  msg.y = y;
  msg.width = width;
  msg.height = height;
  msg.confidence = confidence;
  pub1.publish(msg);

  return;
}

bool Main::newImageReceived()
{
  semaphore.lock();
  return true;
}

// Quando l'ultima confidenza restituita da TLD e' nulla, si invia a tld un'immagine nera tranne
// in una search_area incentrata nel centro della bounding box restituita dall'odometria
// - spostarlo in 'imageReceivedCB' e farlo per ogni immagine?
// - inserire un controllo sul tempo
void Main::getLastImageFromBuffer()
{
  mutex.lock();
  img_header = img_buffer_ptr->header;
  img = img_buffer_ptr->image; // img e' di tipo 'cv::Mat'

  cv::Mat roi_img;

  // per debug! Togliere!
  //odom_bb.x = 400;
  //odom_bb.y = 393;
  //odom_bb.width = 36;
  //odom_bb.height = 36;
  //odom_bb.confidence = 1.0f;
  //pub_odom_rect.publish(odom_bb);

  // Se la confidenza e' nulla, l'oggetto e' uscito dalla scena, oppure e' coperto, se e' coperto gli si dice di cercare in una sotto-search_area data dalla bounding box dell'odometria
  // Controlliamo anche di avere ricevuto le informazioni riguardanti la camera
  if(tld && tld->currConf == 0 && !out_of_view && camera_info_received && odom_bb.x != -1 && kalman_has_converged)
  //if(camera_info_received)
  {
    ROS_WARN("Viene passata la maschera!");

    // l'area di ricerca e' due volte quella della bounding box
    // nel caso la estendersi a sinistra non sia possibile, ci estendiamo a destra
    int accum_width = -1;
    // nel caso la estendersi verso l'alto non sia possibile, ci estendiamo verso l'alto
    int accum_height = -1;
    // nel caso in cui estendersi verso destra non sia possibile, e invece ci sia ancora spazio a sinistra, ci estendiamo a sinistra
    //int accum_x = -1;
    // nel caso in cui estendersi verso il basso non sia possibile, e invece ci sia ancora spazio in alto, ci estendiamo verso l'alto
    //int accum_y = -1;

    int n = 2;

    // Calcolo del centro della bounding box dell'odometria
    int odom_bb_center_x = odom_bb.x + odom_bb.width/2;
    int odom_bb_center_y = odom_bb.y + odom_bb.height/2;

    int search_area_x = odom_bb_center_x - (n * odom_bb.width/2);

    if(search_area_x < 0)
    {
      ROS_WARN("x < 0");
      //accum_width = search_area_x * (-1);
      search_area_x = 0;
    }

    int search_area_y = odom_bb_center_y - (n * odom_bb.height/2);

    if(search_area_y < 0)
    {
      ROS_WARN("y < 0");
      //accum_height = search_area_y * (-1);
      search_area_y = 0;
    }

    int search_area_width = (odom_bb.width * n) + accum_width;

    if((search_area_width+search_area_x) > img.cols)
    {
      ROS_WARN("width+x > img_cols");
      //accum_x = search_area_width - img.cols;
      search_area_width = img.cols - search_area_x;
    }

    int search_area_height = (odom_bb.height * n) + accum_height;

    if((search_area_height+search_area_y) > img.rows)
    {
      ROS_WARN("height+y > img_rows");
      //accum_y = search_area_height - img.rows;
      search_area_height = img.rows - search_area_y;
    }

    // TODO: ri-abilitare questi controlli
    ////if(search_area_x > 0)
    ////{
    ////search_area_x = search_area_x - accum_x;

    ////if(search_area_x < 0)
    ////{
    ////search_area_x = 0;
    ////}
    ////}

    ////if(search_area_y > 0)
    ////{
    ////search_area_y = search_area_y - accum_y;

    if(search_area_y < 0)
    {
    search_area_y = 0;
    }
    //}

    // Calcolo della ROI
    //TODO: aggiungere altri controlli
    if(search_area_x > 0 && search_area_y > 0 && search_area_width > 1 && search_area_height > 1)
    {
      //TODO: rimuovere operazioni inutili
      cv::Rect search_area = cv::Rect(search_area_x, search_area_y, search_area_width, search_area_height);
      //cv::Rect search_area = cv::Rect(300, 300, 90, 90);
      ROS_INFO("search_area x = %d, search area y = %d, search area width = %d, search area height = %d", search_area.x,
               search_area.y, search_area.width, search_area.height);
      cv::Mat clone_img = img.clone();
      ROS_INFO("clone_img rows = %d, clone_img cols = %d.", clone_img.rows, clone_img.cols);

      cv::Mat black_mat(img.rows, img.cols, CV_8UC3, cv::Scalar(0, 0, 0));
      ROS_INFO("black_mat rows = %d, black_mat cols = %d.", black_mat.rows, black_mat.cols);
      cv::Mat black_mat_roi = black_mat(search_area);
      ROS_INFO("black_mat rows = %d, black_mat cols = %d.", black_mat.rows, black_mat.cols);
      ROS_INFO("black_mat_roi rows = %d, black_mat_roi cols = %d.", black_mat_roi.rows, black_mat_roi.cols);
      clone_img(search_area).copyTo(black_mat_roi);
      ROS_INFO("black_mat rows = %d, black_mat cols = %d.", black_mat.rows, black_mat.cols);

      // Viene inviata l'immagine sul topic per la gui (serve solo per il debug visivo in realta')
      cv_bridge::CvImage redirected_img_msg;
      redirected_img_msg.header = img_header;
      redirected_img_msg.encoding = img_buffer_ptr->encoding;
      redirected_img_msg.image = black_mat;
      ROS_WARN("Maschera inviata!");
      pub_redirected_image.publish(redirected_img_msg.toImageMsg()); // Viene pubblicata la ROI ingrandita all'inverosimile

      redirected_img_msg.image.copyTo(img);
    }
  }

  //ROS_WARN("PRIMA DELLA CONVERSIONE");
  cv::cvtColor(img, gray, CV_BGR2GRAY);
  //cv::cvtColor(roi_img, gray, CV_BGR2GRAY);

  img_buffer_ptr.reset();
  mutex.unlock();

  return;
}

void Main::clearBackground()
{
  tld::ForegroundDetector* fg = tld->detectorCascade->foregroundDetector;

  if(fg->bgImg.empty())
  {
    gray.copyTo(fg->bgImg);
  }
  else
  {
    fg->bgImg.release();
  }

  return;
}

void Main::stopTracking()
{
  if(state == STOPPED)
  {
    state = TRACKING;
  }
  else
  {
    state = STOPPED;
  }

  return;
}

void Main::toggleLearning()
{
  tld->learningEnabled = !tld->learningEnabled;
  ROS_INFO("LearningEnabled: %d\n", tld->learningEnabled);

  return;
}

void Main::alternatingMode()
{
  tld->alternating = !tld->alternating;
  ROS_INFO("Alternating: %d\n", tld->alternating);

  return;
}

void Main::exportModel()
{
  ros::NodeHandle np("~");
  np.getParam("modelExportFile", modelExportFile);
  //tld->learningEnabled = false;
  tld->writeToFile(modelExportFile.c_str());
  ROS_INFO("Exporting model %s", modelExportFile.c_str());

  return;
}

void Main::importModel()
{
  ros::NodeHandle np("~");
  np.getParam("modelImportFile", modelImportFile);
  loadModel = true;
  state = TRACKER_INIT;

  return;
}

void Main::reset()
{
  ROS_INFO("ros_opentld: sto effettuando il reset!");

  correctBB = false;
  state = INIT;

  return;
}

cv::Rect Main::faceDetection()
{
  std::vector<cv::Rect> faces;

  while(faces.empty())
  {
    if(newImageReceived())
    {
      getLastImageFromBuffer();
    }

    cv::equalizeHist(gray, gray);
    face_cascade.detectMultiScale(gray, faces, 1.1, 2, 0 | CV_HAAR_SCALE_IMAGE, cv::Size(30, 30));
  }

  return faces[0];
}
