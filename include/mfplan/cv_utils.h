#ifndef CV_UTILS_H
#define CV_UTILS_H

#include <optional>
#include <string>

#include <lemon/dim2.h>

#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/geometric/PathGeometric.h>

#include <opencv2/opencv.hpp>

namespace mfplan {

void viz_path(std::optional<ompl::geometric::PathGeometric> path, cv::Mat img) {
  if (!path.has_value()) { return; }
  cv::Scalar color(255, 0, 0);
  for (std::size_t i = 0; i < path->getStateCount(); ++i) {
    double x = path->getState(i)->as<ompl::base::SE2StateSpace::StateType>()->getX();
    double y = path->getState(i)->as<ompl::base::SE2StateSpace::StateType>()->getY();
    cv::Point ctr(x, y);
    cv::circle(img, ctr, 2, color, -1);
  }
}

void viz_point(lemon::dim2::Point<double> coords, cv::Mat img) {
  cv::Scalar color(0, 255, 0);
  cv::Point ctr(coords.x, coords.y);
  cv::circle(img, ctr, 5, color, -1);
}

// convert numeric image type to string for debug
std::string getImgType(int imgTypeInt) {
  // 7 base types, wit h five channel options each (none or C1, ..., C4)
  int numImgTypes = 35;

  int enum_ints[] = {
    CV_8U,  CV_8UC1,  CV_8UC2,  CV_8UC3,  CV_8UC4,
    CV_8S,  CV_8SC1,  CV_8SC2,  CV_8SC3,  CV_8SC4,
    CV_16U, CV_16UC1, CV_16UC2, CV_16UC3, CV_16UC4,
    CV_16S, CV_16SC1, CV_16SC2, CV_16SC3, CV_16SC4,
    CV_32S, CV_32SC1, CV_32SC2, CV_32SC3, CV_32SC4,
    CV_32F, CV_32FC1, CV_32FC2, CV_32FC3, CV_32FC4,
    CV_64F, CV_64FC1, CV_64FC2, CV_64FC3, CV_64FC4};

  std::string enum_strings[] = {
    "CV_8U",  "CV_8UC1",  "CV_8UC2",  "CV_8UC3",  "CV_8UC4",
    "CV_8S",  "CV_8SC1",  "CV_8SC2",  "CV_8SC3",  "CV_8SC4",
    "CV_16U", "CV_16UC1", "CV_16UC2", "CV_16UC3", "CV_16UC4",
    "CV_16S", "CV_16SC1", "CV_16SC2", "CV_16SC3", "CV_16SC4",
    "CV_32S", "CV_32SC1", "CV_32SC2", "CV_32SC3", "CV_32SC4",
    "CV_32F", "CV_32FC1", "CV_32FC2", "CV_32FC3", "CV_32FC4",
    "CV_64F", "CV_64FC1", "CV_64FC2", "CV_64FC3", "CV_64FC4"};

  for (int i=0; i<numImgTypes; i++) {
    if (imgTypeInt == enum_ints[i]) return enum_strings[i];
  }
  return "unknown image type";
}

} // namespace mfplan

#endif