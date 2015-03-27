/*
 *
 *  Copyright (C) 1997-2009 JDERobot Developers Team
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see http://www.gnu.org/licenses/. 
 *
 *  Authors : Rubén González Barriada <ruben.gbarriada@gmail.com>
 *						Alejandro Hernández Cordero <ahcorde@gmail.com> 
 *            Óscar Javier García Baudet <oscar.robotica@linaresdigital.com>
 *
 */

#include <Ice/Ice.h>

#include <jderobot/camera.h>
#include <visionlib/colorspaces/colorspacesmm.h>
#include "viewer.h"

static std::string supported_format_rgb8("RGB8");
static std::string supported_format_nv21("NV21");

int main(int argc, char** argv) {
  opencvdemo::Viewer viewer;
  Ice::CommunicatorPtr ice_communicator;
  Ice::ObjectPrx base;
  jderobot::CameraPrx camera_proxy = 0;
  jderobot::ImageDataPtr image_data;
  cv::Mat image;

  /* Initialize Ice and load parameters from command line */
  try {
    ice_communicator = Ice::initialize(argc, argv);
    base = ice_communicator->propertyToProxy("Opencvdemo.Camera.Proxy");
    if (0 == base)
      throw "Could not create proxy";
  } catch (const Ice::Exception& ex) {
    /* Show Ice generated error */
    std::cerr << "ERROR: " << ex << std::endl;
    return 1;
  } catch (const char* msg) {
    std::cerr << msg << std::endl;
    return 1;
  }

  /* While GUI is visible (user didn't close it) */
  while (viewer.isVisible()) {
    /* Try to get next frame */
    try {
      /* Get casted proxy to access all Camera methods */
      if (camera_proxy == 0) {
        camera_proxy = jderobot::CameraPrx::checkedCast(base);
        if (camera_proxy == 0) {
          std::cerr << "ERROR: Invalid proxy" << std::endl;
          return 1;
        }
      }
      /* Get next frame */
      image_data = camera_proxy->getImageData();
    } catch (const Ice::Exception& ex) {
      /* Show an error in stderr */
      std::cerr << "ICE error: " << ex << std::endl;
      /* Show the default "Missing Image" icon instead input image */
      viewer.DisplayError();
      /* Wait 250ms before trying again */
      usleep(250000);
      continue;
    }

    /* Check if format is supported by colorspaces */
    colorspaces::Image::FormatPtr format_string =
        colorspaces::Image::Format::searchFormat(
            image_data->description->format);
    if (!format_string)
      throw "Format not supported";

    /* Check if format is supported by our application (else we need to convert to RGB8) */
    if (supported_format_rgb8.compare(image_data->description->format) == 0) {
      /* Create a OpenCV Mat from RGB8 data (8 bits per pixel) received */
      image = cv::Mat(image_data->description->height,
                      image_data->description->width, CV_8UC3,
                      &(image_data->pixelData[0]));
    } else if (supported_format_nv21.compare(image_data->description->format)
        == 0) {
      /* Convert Android's NV21 image format in GTK compatible format */
      colorspaces::ImageNV21 android_image(image_data->description->width,
                                           image_data->description->height,
                                           &(image_data->pixelData[0]));
      /* Create a new image converting android_image from NV21 to RGB8 format */
      colorspaces::ImageRGB8 image2(android_image);
      /* Convert old IplImage format used in colorspaces to newer cv::Mat */
      image = cv::Mat(image2);
    } else {
      throw "Format not implemented";
    }

    /* Display and process input image */
    viewer.Display(image);
  }

  /* Free ICE resources */
  if (ice_communicator)
    ice_communicator->destroy();

  return 0;
}
