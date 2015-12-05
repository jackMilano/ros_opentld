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
            last_tld_curr_bb_x = tld_curr_bb_x;
            last_tld_curr_bb_y = tld_curr_bb_y;
            tld_curr_bb_x = tld->currBB->x;
            tld_curr_bb_y = tld->currBB->y;
            tld_curr_bb_width = tld->currBB->width;
            tld_curr_bb_height = tld->currBB->height;
            sendTrackedObject(tld->currBB->x, tld->currBB->y, tld->currBB->width, tld->currBB->height, tld->currConf);
          }
          else
          {
            last_tld_curr_bb_x = 1;
            last_tld_curr_bb_y = 1;
            tld_curr_bb_x = 1;
            tld_curr_bb_y = 1;
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

// Riceve la posizione combinata e la converte in 2D sul piano
void Main::positionFusedReceivedCB(const nav_msgs::OdometryConstPtr& fused_odom_msg)
{

  if(!transform_listener.waitForTransform(fused_odom_msg->header.frame_id, cam_frame_id, fused_odom_msg->header.stamp,
                                          ros::Duration(WAIT_TRANS_TIME)))
  {
    ROS_ERROR("ros_open_tld: wait for transform %s --> %s timed out!!", fused_odom_msg->header.frame_id.c_str(),
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
  odom_pose.header = fused_odom_msg->header; // frame_id = "fused_odom_msg"
  odom_pose.pose = fused_odom_msg->pose.pose;
  geometry_msgs::PoseStamped cam_odom; // frame_id = "kinect2_rgb_optical_frame"
  transform_listener.transformPose(cam_frame_id, odom_pose, cam_odom);

  // Calcolo posizione sul piano immagine (sistema di riferimento camera) dell'odometria.
  int fusion_img_x = 0, fusion_img_y = 0;
  odomToImgPlan(cam_odom.pose.position.x, cam_odom.pose.position.y, cam_odom.pose.position.z, fusion_img_x, fusion_img_y);

  last_fusion_bb_center_x = fusion_bb_center_x;
  last_fusion_bb_center_y = fusion_bb_center_y;
  fusion_bb_center_x = fusion_img_x;
  fusion_bb_center_y = fusion_img_y;

  // PUBBLICAZIONE DELLA BOUNDING BOX CORRISPONDENTE ALLA POSIZIONE COMBINATA:
  // - serve per i test
  // - la bounding box viene calcolata come se avesse il centro nella posizione dell'odometria
  //   (2D, sistema di riferimento camera)
  // - la larghezza e l'altezza della bounding box corrispondono a quelle dell'ultima bounding box valida
  fusion_bb.header.seq = img_header.seq;
  fusion_bb.header.stamp = ros::Time::now();
  fusion_bb.header.frame_id = cam_frame_id;

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

  fusion_bb.x = fusion_img_x - static_cast<int>(round(static_cast<double>(tld_curr_bb_width) / 2.0));

  if(fusion_bb.x < 0)
  {
    ROS_WARN("Attenzione! La coordinata 'x' della Bounding Box dell'odometria calcolata è negativa. Viene azzerata.");
    fusion_bb.x = 0;
  }

  fusion_bb.y = fusion_img_y - static_cast<int>(round(static_cast<double>(tld_curr_bb_height) / 2.0));

  if(fusion_bb.y < 0)
  {
    ROS_WARN("Attenzione! La coordinata 'y' della Bounding Box dell'odometria calcolata è negativa. Viene azzerata.");
    fusion_bb.y = 0;
  }

  fusion_bb.width = tld_curr_bb_width;
  fusion_bb.height = tld_curr_bb_height;

  last_fusion_bb_width = fusion_bb_width;
  last_fusion_bb_height = fusion_bb_height;
  fusion_bb_width = fusion_bb.width;
  fusion_bb_height = fusion_bb.height;

  fusion_bb.confidence = 1.0f;

  if((fusion_img_x < 0) || (fusion_img_x + fusion_bb.width - 2) > img_width || (fusion_img_y < 0)
      || (fusion_img_y + fusion_bb.height - 2) > img_height)
  {
    if(fusion_img_x < 0)
    {
      ROS_WARN("Out of view. fusion_img_x = %d", fusion_img_x);
    }

    if((fusion_img_x + tld_curr_bb_width - 2) > img_width)
    {
      ROS_WARN("Out of view. fusion_img_x = %d, img_width = %d, tld_curr_bb_width = %d", fusion_img_x, img_width,
               tld_curr_bb_width);
    }

    if(fusion_img_y < 0)
    {
      ROS_WARN("Out of view. fusion_img_y = %d", fusion_img_y);
    }

    if((fusion_img_y + tld_curr_bb_height - 2) > img_height)
    {
      ROS_WARN("Out of view. fusion_img_y = %d, img_height = %d, tld_curr_bb_height = %d", fusion_img_y, img_height,
               tld_curr_bb_height);
    }

    out_of_view = true;
  }
  else
  {
    out_of_view = false;
  }

  //ROS_INFO("ros_tld_tracker: bounding box odometria inviata");
  pub_odom_rect.publish(fusion_bb);

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

// Quando l'ultima confidenza restituita da TLD è nulla, si invia a TLD un'immagine nera tranne
// in una ROI incentrata nel centro della bounding box corrispondente alla posizione combinata
// - spostarlo in 'imageReceivedCB' e farlo per ogni immagine?
// - inserire un controllo sul tempo
void Main::getLastImageFromBuffer()
{
  mutex.lock();
  img_header = img_buffer_ptr->header;
  img = img_buffer_ptr->image; // img e' di tipo 'cv::Mat'

  cv::Mat roi_img;

  // Controlliamo innanzitutto di avere ricevuto i parametri intrinseci della camera, che i valori
  // della posizione fused siano validi e che kalman abbia converso.
  if(tld && camera_info_received && last_fusion_bb_center_x != -1 && kalman_has_converged)
  {
    //int fusion_bb_center_x = fusion_bb.x + fusion_bb.width / 2;
    //int fusion_bb_center_y = fusion_bb.y + fusion_bb.height / 2;

    // Se la confidenza è nulla, il target è occluso oppure è uscito dalla scena.
    // In caso di occlusione si calcola una ROI dell'immagine, avente come centro il centro
    // della bounding box della posizione combinata
    if(tld->currConf == 0)
    {
      if(!out_of_view)
      {
        ROS_WARN("ros_opentld: target occluso! viene calcolata la ROI!");

        // l'area di ricerca è 'n' volte quella della bounding box
        // nel caso in cui estendersi a sinistra non sia possibile, ci estendiamo a destra
        //int accum_width = -1;
        // nel caso in cui estendersi verso l'alto non sia possibile, ci estendiamo verso il basso
        //int accum_height = -1;

        const int n = 3;

        int search_area_x = last_fusion_bb_center_x - (n * last_fusion_bb_width / 2);

        if(search_area_x < 0)
        {
          ROS_WARN("ros_opentld: x < 0");
          search_area_x = 0;
        }

        int search_area_y = last_fusion_bb_center_y - (n * last_fusion_bb_height / 2);

        if(search_area_y < 0)
        {
          ROS_WARN("ros_opentld: y < 0");
          search_area_y = 0;
        }

        //int search_area_width = (fusion_bb.width * n) + accum_width;
        int search_area_width = (last_fusion_bb_width * n);

        if((search_area_width + search_area_x) > img.cols)
        {
          ROS_WARN("ros_opentld: width+x > img_cols");
          search_area_width = img.cols - search_area_x;
        }

        //int search_area_height = (fusion_bb.height * n) + accum_height;
        int search_area_height = (last_fusion_bb_height * n);

        if((search_area_height + search_area_y) > img.rows)
        {
          ROS_WARN("ros_opentld: height+y > img_rows");
          search_area_height = img.rows - search_area_y;
        }

        // Calcolo della ROI
        //TODO: aggiungere altri controlli
        if(search_area_x > 0 && search_area_y > 0 && search_area_width > 1 && search_area_height > 1)
        {
          //TODO: rimuovere operazioni inutili
          cv::Rect search_area = cv::Rect(search_area_x, search_area_y, search_area_width, search_area_height);
          cv::Mat clone_img = img.clone();

          cv::Mat black_mat(img.rows, img.cols, CV_8UC3, cv::Scalar(0, 0, 0));
          cv::Mat black_mat_roi = black_mat(search_area);
          clone_img(search_area).copyTo(black_mat_roi);

          // Viene inviata l'immagine sul topic per la GUI (serve solo per il debug visivo in realtà)
          cv_bridge::CvImage redirected_img_msg;
          redirected_img_msg.header = img_header;
          redirected_img_msg.encoding = img_buffer_ptr->encoding;
          redirected_img_msg.image = black_mat;
          //pub_redirected_image.publish(redirected_img_msg.toImageMsg());

          ROS_WARN("ros_opentld: Maschera inviata!");
          redirected_img_msg.image.copyTo(img);
        }
      }
      else
      {
        // In caso di 'out of view' viene inviata una matrice totalmente nera, in modo che il target
        // non venga cercato nella scena
        cv::Mat black_mat(img.rows, img.cols, CV_8UC3, cv::Scalar(0, 0, 0));

        // Viene inviata l'immagine sul topic per la GUI (serve solo per il debug visivo in realtà)
        cv_bridge::CvImage redirected_img_msg;
        redirected_img_msg.header = img_header;
        redirected_img_msg.encoding = img_buffer_ptr->encoding;
        redirected_img_msg.image = black_mat;
        //pub_redirected_image.publish(redirected_img_msg.toImageMsg());

        ROS_WARN("ros_opentld: Out of View! L'immagine inviata è totalmente nera!");
        redirected_img_msg.image.copyTo(img);
      }
    }
    else
    {
      // Controlliamo che TLD non abbia fatto movimenti bruschi da un frame all'altro.

      const int max_dist = 10 * sqrt(pow(36, 2) + pow(36, 2));

      if(tld_curr_bb_x != 1)
      {
        //int last_tld_bb_center_x = last_tld_curr_bb_x + last_tld_curr_bb_width / 2;
        //int last_tld_bb_center_y = last_tld_curr_bb_y + last_tld_curr_bb_height / 2;
        int tld_bb_center_x = tld_curr_bb_x + tld_curr_bb_width / 2;
        int tld_bb_center_y = tld_curr_bb_y + tld_curr_bb_height / 2;

        //const int time_movement = sqrt(pow(last_tld_bb_center_x - tld_bb_center_x, 2) + pow(last_tld_bb_center_y - tld_bb_center_y, 2));
        int space_movement = sqrt(pow(last_fusion_bb_center_x - tld_bb_center_x, 2) + pow(last_fusion_bb_center_y - tld_bb_center_y, 2));

        // Quando la posizione di TLD e quella combinata divergono di un ampia distanza, si effettua la ricerca in una ROI
        // TODO: funzione per il calcolo ed invio della ROI, invece che riscrivere lo stesso codice in due posti diversi
        //if(time_movement > max_dist || space_movement > max_dist)
        if(space_movement > max_dist)
        {
          ROS_INFO("ros_open_tld: divergenza di posizione tra posizione combinata e tld. ROI inviata.");
          const int n = 4; // Più grande che rispetto ad occlusione (3)

          int search_area_x = last_fusion_bb_center_x - (n * last_fusion_bb_width / 2);

          if(search_area_x < 0)
          {
            ROS_WARN("ros_opentld: x < 0");
            search_area_x = 0;
          }

          int search_area_y = last_fusion_bb_center_y - (n * last_fusion_bb_height / 2);

          if(search_area_y < 0)
          {
            ROS_WARN("ros_opentld: y < 0");
            search_area_y = 0;
          }

          //int search_area_width = (fusion_bb.width * n) + accum_width;
          int search_area_width = (last_fusion_bb_width * n);

          if((search_area_width + search_area_x) > img.cols)
          {
            ROS_WARN("ros_opentld: width+x > img_cols");
            search_area_width = img.cols - search_area_x;
          }

          //int search_area_height = (fusion_bb.height * n) + accum_height;
          int search_area_height = (last_fusion_bb_height * n);

          if((search_area_height + search_area_y) > img.rows)
          {
            ROS_WARN("ros_opentld: height+y > img_rows");
            search_area_height = img.rows - search_area_y;
          }

          // Calcolo della ROI
          //TODO: aggiungere altri controlli
          if(search_area_x > 0 && search_area_y > 0 && search_area_width > 1 && search_area_height > 1)
          {
            //TODO: rimuovere operazioni inutili
            cv::Rect search_area = cv::Rect(search_area_x, search_area_y, search_area_width, search_area_height);
            cv::Mat clone_img = img.clone();

            cv::Mat black_mat(img.rows, img.cols, CV_8UC3, cv::Scalar(0, 0, 0));
            cv::Mat black_mat_roi = black_mat(search_area);
            clone_img(search_area).copyTo(black_mat_roi);

            // Viene inviata l'immagine sul topic per la GUI (serve solo per il debug visivo in realtà)
            cv_bridge::CvImage redirected_img_msg;
            redirected_img_msg.header = img_header;
            redirected_img_msg.encoding = img_buffer_ptr->encoding;
            redirected_img_msg.image = black_mat;
            //pub_redirected_image.publish(redirected_img_msg.toImageMsg());

            ROS_WARN("ros_opentld: Maschera inviata!");
            redirected_img_msg.image.copyTo(img);
          }
        }
      }
    }
  }

  cv::cvtColor(img, gray, CV_BGR2GRAY);

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
