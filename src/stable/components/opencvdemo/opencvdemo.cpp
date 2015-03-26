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

static std::string supported_format_1("RGB8");

int main(int argc, char** argv) {
  int status;
  opencvdemo::Viewer viewer;
  Ice::CommunicatorPtr ic;
  Ice::ObjectPrx base;
  jderobot::CameraPrx cprx = 0;
  jderobot::ImageDataPtr data;

  /* Initialize Ice and load parameters from command line */
  try {
    ic = Ice::initialize(argc, argv);
    base = ic->propertyToProxy("Opencvdemo.Camera.Proxy");
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
      if (cprx == 0) {
        cprx = jderobot::CameraPrx::checkedCast(base);
        if (0 == cprx)
          throw "Invalid proxy";
      }
      /* Get next frame */
      data = cprx->getImageData();
    } catch (const Ice::Exception& ex) {
      /* Show an error in stderr, show a error icon and wait 250ms before try again */
      std::cerr << "Unexpected run‑time error: " << ex << std::endl;
      viewer.DisplayError();
      usleep(250000);
      continue;
    } catch (const char* msg) {
      std::cerr << msg << std::endl;
      return 1;
    }

    std::cout << "Format: " << data->description->format << std::endl;
    colorspaces::Image::FormatPtr fmt =
        colorspaces::Image::Format::searchFormat(data->description->format);

    if (!fmt)
      throw "Format not supported";
    char * data1;
    cv::Mat image;
    if (supported_format_1.compare(data->description->format) == 0) {
      data1 = new char[data->description->width
          * data->description->height * 3];
      memcpy((unsigned char *) data1, &(data->pixelData[0]),
             data->description->width * data->description->height * 3);
      image = cv::Mat(data->description->height, data->description->width,
                    CV_8UC3, &(data->pixelData[0]));
      cv::Mat image2(data->description->height, data->description->width, CV_8UC3,
                     data1);
    } else {
      throw "Format not implemented";
    }

    // Selecting the operation
    //viewer.selection(image2);

    // Displaying the images
    viewer.Display(image);
    delete data1;

  }

  if (ic)
    ic->destroy();
  return status;
}
