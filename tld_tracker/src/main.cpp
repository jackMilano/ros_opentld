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
        //ROS_DEBUG("'gray.step' = %d.", gray.step);

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

        ROS_INFO("Starting at %d %d %d %d\n", target_bb.x, target_bb.y, target_bb.width, target_bb.height);

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
            sendTrackedObject(tld->currBB->x, tld->currBB->y, tld->currBB->width, tld->currBB->height, tld->currConf);
          }
          else
          {
            sendTrackedObject(1, 1, 1, 1, 0.0);
          }
        }
      }

      break;

    // case TRACKING_FALSE_POSITIVE:
    // if( correctBB && newBB )
    // {
    // sendTrackedObject(new_target_bb.x, new_target_bb.y, new_target_bb.width, new_target_bb.height, 1.0);
    // ROS_INFO("Coordinate nuova Bounding Box: %d %d %d %d.\n", new_target_bb.x, new_target_bb.y, new_target_bb.width, new_target_bb.height);

    // // Viene calcolata, a partire dalla bounding box corrispondente all'odometria,
    // // la ROI nella quale ricercare il target.
    // cv::Rect roi;
    // roi.x = new_target_bb.x - new_target_bb.width;
    // if( roi.x < 0 )
    // {
    // roi.x = 0;
    // }
    // roi.y = new_target_bb.y - new_target_bb.height;
    // if( roi.y < 0 )
    // {
    // roi.y = 0;
    // }
    // roi.width = 3 * new_target_bb.width;
    // if( (roi.x + roi.width) > tld->detectorCascade->imgWidth )
    // {
    // roi.width = tld->detectorCascade->imgWidth;
    // }
    // roi.height = 3 * new_target_bb.height;
    // if( (roi.y + roi.height) > tld->detectorCascade->imgHeight )
    // {
    // roi.height = tld->detectorCascade->imgHeight;
    // }

    // cv::Mat img_roi = img(roi);
    // cv::Mat gray_roi = gray(roi);

    // // Devo cambiare le dimensioni dell'immagine per il 'detectorCascade'?
    // // tld->detectorCascade->imgWidth = gray_roi.cols;
    // // tld->detectorCascade->imgHeight = gray_roi.rows;
    // // tld->detectorCascade->imgWidthStep = gray_roi.step;
    // // Questi sono i passaggi fatti in 'tld->selectObject':
    // // 1. tld->detectorCascade->release()
    // // - cancella i risultati precedenti e quindi la memoria dell'oggetto target e dell'immagine
    // // 2. tld->detectorCascade->init()
    // // - prima di chiamare questo membro bisogna settare 'imgWidth' e 'imgHeight'
    // // - chiama 'initWindowsAndScales()'
    // // - chiama 'initWindowOffsets()'
    // // - inizializza le componenti del detector cascade
    // // - inizializza il 'varianceFilter'
    // // - inizializza il 'ensembleClassifier'
    // // - inizializza il 'nnClassifier'
    // // 3. tld->initialLearning()
    // // - inizializza l'apprendimento delle tre componenti del detector cascade

    // // chiama 'detectorCascade->detect' che dipende dalla dimensione dell'immagine iniziale
    // tld->processROIImage(img_roi);

    // if(showOutput)
    // {
    // if(tld->currBB != NULL)
    // {
    // sendTrackedObject(tld->currBB->x, tld->currBB->y, tld->currBB->width, tld->currBB->height, tld->currConf);

    // // reInit = false;
    // newBB = false; // Non appena si è trovata una bounding box valida si esce da questo stato.
    // STATE = TRACKING;
    // }
    // else
    // {
    // ROS_WARN("TRACKING_FALSE_POSITIVE: target non trovato!");
    // sendTrackedObject(1, 1, 1, 1, 0.0);
    // }
    // }


    // }
    // else // Aspettiamo finché non è arrivata la bounding box.
    // {
    // ros::Duration(1.0).sleep();
    // ROS_INFO("Sto aspettando la nuova BB.");
    // }
    // break;
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

void Main::targetReceivedCB(const tld_msgs::TargetConstPtr& msg)
{
  reset(); //XXX: ogni volta che viene chiamata questa callback viene chiamato anche 'reset()'!!!
  ROS_ASSERT(msg->bb.x >= 0);
  ROS_ASSERT(msg->bb.y >= 0);
  ROS_ASSERT(msg->bb.width > 0);
  ROS_ASSERT(msg->bb.height > 0);
  ROS_INFO("Bounding Box received");

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
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  correctBB = true;

  return;
}

// void Main::bbReceivedCB(const tld_msgs::BoundingBoxConstPtr & msg)
// {
// ROS_INFO("Dentro 'bbReceivedCB'!");
// ROS_ASSERT(msg->bb.x >= 0);
// ROS_ASSERT(msg->bb.y >= 0);
// ROS_ASSERT(msg->bb.width > 0);
// ROS_ASSERT(msg->bb.height > 0);

// new_target_bb.x = msg->x;
// new_target_bb.y = msg->y;
// new_target_bb.width = msg->width;
// new_target_bb.height = msg->height;

// // correctBB = true;
// newBB = true;
// reInit = true;

// return;
// }

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

  case 'z':
    ROS_INFO("Comando ricevuto: 'z'!");
    // reinit();
    // state = TRACKING_FALSE_POSITIVE;
    // reInit = false;
    // newBB = false;
    reset();
    break;

  default:
    break;
  }

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

void Main::getLastImageFromBuffer()
{
  mutex.lock();
  img_header = img_buffer_ptr->header;
  img = img_buffer_ptr->image;

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
  ROS_INFO("Sto effettuando il reset.");

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

