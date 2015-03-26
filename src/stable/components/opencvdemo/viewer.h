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
 *            Alejandro Hernández Cordero <ahcorde@gmail.com>
 *            Óscar Javier García Baudet <oscar.robotica@linaresdigital.com>
 *
 */

#ifndef JDEROBOT_COMPONENTS_OPENCVDEMO_VIEWER_H_
#define JDEROBOT_COMPONENTS_OPENCVDEMO_VIEWER_H_

#include <gtkmm.h>
#include <glade/glade.h>
#include <libglademm.h>
#include <opencv2/opencv.hpp>

/* Trigonometric utilities */
#define PI 3.14159265359
#define DEGTORAD (PI / 180.0)
#define SQUARE(a) (a)*(a)

namespace opencvdemo {

class Viewer {
 public:
  Viewer();
  ~Viewer();

  // function that actually displays the images
  void DisplayError();
  void Display(cv::Mat image);
  bool isVisible();
  bool OnClickedEventBox(GdkEventButton* event);


 private:

  Glib::RefPtr<Gnome::Glade::Xml> ref_xml_;

  Gtk::Image* gtk_image_in_;  // input image
  Gtk::Image* gtk_image_out_;  // output image
  Gtk::Window* main_window_;
  Gtk::Main gtk_main_;

  //IplImage* imagenO;
  cv::Mat imagenO_;
  pthread_mutex_t mutex_;

  // Horizontal slidebars
  Gtk::HScale* scale_sobel_;
  Gtk::HScale* scale_canny_;
  Gtk::HScale* scale_hough_threshold_;
  Gtk::HScale* scale_hough_long_;
  Gtk::HScale* scale_hough_gap_;

  // Vertical slidebars
  Gtk::VScale* scale_h_max_;
  Gtk::VScale* scale_h_min_;
  Gtk::VScale* scale_v_max_;
  Gtk::VScale* scale_v_min_;
  Gtk::VScale* scale_s_max_;
  Gtk::VScale* scale_s_min_;

  Gtk::EventBox* eventbox_;

  // Check buttons that implement the filters
  Gtk::CheckButton* button_canny_;
  Gtk::CheckButton* button_sobel_;
  Gtk::CheckButton* button_laplace_;
  Gtk::CheckButton* button_hough_;
  Gtk::CheckButton* button_harris_;
  Gtk::CheckButton* button_default_;
  Gtk::CheckButton* button_gray_;
  Gtk::CheckButton* button_flow_;
  Gtk::CheckButton* button_conv_;
  Gtk::CheckButton* button_pyramid_;
  Gtk::CheckButton* button_color_;
  Gtk::CheckButton* button_houghcircles_;

  // Selection of Method
  Gtk::ComboBox* combobox_hough_;
  Gtk::ComboBox* combobox_conv_;

  // Labels
  Gtk::Label* label_long_;
  Gtk::Label* label_gap_;


  //Filters and Feature detectors
  void selection(cv::Mat image);
  void canny(cv::Mat image);
  void sobel(cv::Mat image);
  void laplace(cv::Mat image);
  void hough(cv::Mat image);
  void hough_circles(cv::Mat image);
  void harris(cv::Mat image);
  void gray(cv::Mat image);
  void flow(cv::Mat image);
  void color(cv::Mat image);
  void conv(cv::Mat image);
  void pyramid(cv::Mat image);
  int valuesOK(double H, double S, double V);

  double getH(double r, double g, double b);
  double getS(double r, double g, double b);
  double getV(double r, double g, double b);

  //Checks if the button has been clicked
  void button_canny_clicked();
  void button_sobel_clicked();
  void button_laplace_clicked();
  void button_hough_clicked();
  void button_hough_circles_clicked();
  void button_harris_clicked();
  void button_default_clicked();
  void button_gray_clicked();
  void button_flow_clicked();
  void button_color_clicked();
  void button_conv_clicked();
  void button_pyramid_clicked();

  // Checkbox control
  int canny_box_;
  int sobel_box_;
  int laplace_box_;
  int harris_box_;
  int hough_box_;
  int houghcircles_box_;
  int def_box_;
  int gray_box_;
  int flow_box_;
  int color_box_;
  int conv_box_;
  int pyramid_box_;

};

}  //namespace

#endif  // JDEROBOT_COMPONENTS_OPENCVDEMO_VIEWER_H_
